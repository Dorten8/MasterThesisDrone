#!/usr/bin/env python3
"""
MCAP Unified Pass Segmenter
===========================
Post-flight utility that physically slices a full raw flight recording into
separate, self-contained pass-level .mcap files.

Features:
- Dual Slicing Modes: Supports both robust event-driven and legacy coordinate-based segmentation.
- Automated Fallback: Defaults to scanning for `/flight_director/active_waypoint` transition events,
  but automatically falls back to MoCap coordinate-based bounding if no event topics are found.
- MCAP Self-Healing: Unlimited record-size StreamReader recovery for interrupted SSHFS streams.
- Approved Cutoff Enforcement: Safely skips legacy test files from before 2026-05-24.

Usage:
    python3 -m dev_logs.analysis.database.db_mcap_event_segmenter                           # Auto-process all approved flights
    python3 -m dev_logs.analysis.database.db_mcap_event_segmenter <path_to_flight_folder>    # Auto-process specific folder
    python3 -m dev_logs.analysis.database.db_mcap_event_segmenter --mode legacy              # Enforce legacy coordinate mode
    python3 -m dev_logs.analysis.database.db_mcap_event_segmenter --mode event               # Enforce event-driven mode
"""
import os
import sys
import glob
import re
import argparse
import pandas as pd
from mcap_ros2.reader import read_ros2_messages
from mcap_ros2.writer import Writer

# ── Dynamic Self-Healing Path Boilerplate ────────────────────────────────────
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from dev_logs.analysis.database.db_loader import load_drone_metadata, load_mcap, build_dataframes
from dev_logs.analysis.kinematics.kin_calculator import compute_velocity, find_waypoint_events
from dev_logs.analysis.database.db_manager import APPROVED_CUTOFF, is_approved_flight

# ── Approved flight cutoff ────────────────────────────────────────────────────
# SSoT lives in db_manager.py — imported above. Local _is_approved() kept as
# a thin wrapper for backward compatibility within this module only.


def _flight_timestamp(folder_name):
    """Extracts YYYYMMDD-HHMM from a flight folder name, or None if not parseable."""
    m = re.search(r'flight_(\d{8}-\d{4})', folder_name)
    return m.group(1) if m else None


def _is_approved(folder_name):
    """Returns True if the flight is at or after the approved cutoff.
    Delegates to the SSoT function in db_manager."""
    return is_approved_flight(folder_name)


def repair_mcap(source_path):
    """
    Recovers a corrupted MCAP by scanning all raw records with an unlimited
    record-size reader (bypasses the bad opcode/length check) and re-writing
    them into a fresh, properly-finalized MCAP file.
    The original file is kept untouched; the repaired file gets .repaired.mcap.

    Returns the repaired file path on success, or None if repair failed.
    """
    from mcap.stream_reader import StreamReader
    from mcap.writer import Writer as RawWriter
    from mcap.records import Message, Channel, Schema

    repaired_path = source_path.replace(".mcap", ".repaired.mcap")
    if os.path.exists(repaired_path):
        print(f"   ♻️  Using existing repaired file: {os.path.basename(repaired_path)}")
        return repaired_path

    print(f"   🔧 Attempting MCAP repair: {os.path.basename(source_path)}")
    try:
        written = 0
        with open(source_path, "rb") as f_in, open(repaired_path, "wb") as f_out:
            raw_writer = RawWriter(f_out)
            raw_writer.start(profile="ros2")
            schema_ids  = {}
            channel_ids = {}

            reader = StreamReader(f_in, record_size_limit=None)
            try:
                for record in reader.records:
                    if isinstance(record, Schema):
                        schema_ids[record.id] = raw_writer.register_schema(
                            name=record.name,
                            encoding=record.encoding,
                            data=record.data
                        )
                    elif isinstance(record, Channel):
                        if record.schema_id in schema_ids:
                            channel_ids[record.id] = raw_writer.register_channel(
                                topic=record.topic,
                                message_encoding=record.message_encoding,
                                schema_id=schema_ids[record.schema_id],
                                metadata=record.metadata
                            )
                    elif isinstance(record, Message):
                        if record.channel_id in channel_ids:
                            raw_writer.add_message(
                                channel_id=channel_ids[record.channel_id],
                                log_time=record.log_time,
                                data=record.data,
                                publish_time=record.publish_time,
                                sequence=record.sequence
                            )
                            written += 1
            except Exception:
                # EndOfFile or corruption mid-record — stop here and finalize what we have
                pass

            raw_writer.finish()

        if written == 0:
            os.remove(repaired_path)
            print(f"   ❌ Repair yielded 0 messages — file is unrecoverable.")
            return None

        print(f"   ✅ Repair complete: {written} messages → {os.path.basename(repaired_path)}")
        return repaired_path

    except Exception as e:
        if os.path.exists(repaired_path):
            os.remove(repaired_path)
        print(f"   ❌ Repair failed: {e}")
        return None


def try_load_mcap(flight_path):
    """
    Tries to load the primary MCAP for a flight, auto-repairing if corrupted.
    Returns (topic_data, bag_start_ns, source_mcap_path) or raises on failure.
    """
    mcap_files = sorted([
        f for f in glob.glob(os.path.join(flight_path, "*.mcap"))
        if "-pass" not in os.path.basename(f)
        and ".repaired" not in os.path.basename(f)
    ])
    if not mcap_files:
        raise FileNotFoundError(f"No primary MCAP files found in: {flight_path}")
    
    source_path = mcap_files[0]

    # Try normal load first
    try:
        topic_data, bag_start_ns = load_mcap(flight_path)
        return topic_data, bag_start_ns, source_path
    except Exception as e:
        print(f"   ⚠️  Normal load failed ({e}), attempting repair...")

    # Repair and retry
    repaired = repair_mcap(source_path)
    if repaired is None:
        raise RuntimeError("MCAP repair failed and no usable data could be recovered.")

    # Read the repaired file using the low-level reader with ROS2 decoder factory
    from mcap.reader import make_reader
    try:
        from mcap_ros2.decoder import DecoderFactory as ROS2DecoderFactory
    except ImportError:
        ROS2DecoderFactory = None

    topic_data = {}
    with open(repaired, "rb") as f:
        reader = make_reader(f, decoder_factories=[ROS2DecoderFactory()] if ROS2DecoderFactory else [])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages():
            t = channel.topic
            if t not in topic_data:
                topic_data[t] = []
            class _Msg:
                def __init__(self, m, r, ns):
                    self.ros_msg = r
                    self.log_time_ns = ns
                    self.channel = m
            topic_data[t].append(_Msg(channel, ros_msg, message.log_time))

    if not topic_data:
        raise RuntimeError("Repaired MCAP contains no readable messages.")

    bag_start_ns = min(m.log_time_ns for msgs in topic_data.values() for m in msgs)
    return topic_data, bag_start_ns, repaired


def has_active_waypoint_topic(flight_path):
    """
    Lightning-fast channel index check to see if `/flight_director/active_waypoint` 
    is recorded in the MCAP channels list. Avoids decoding or reading any messages.
    """
    mcap_files = sorted([
        f for f in glob.glob(os.path.join(flight_path, "*.mcap"))
        if "-pass" not in os.path.basename(f)
        and ".repaired" not in os.path.basename(f)
    ])
    if not mcap_files:
        return False
    
    mcap_path = mcap_files[0]
    
    # Check repaired file first if it already exists
    repaired_path = mcap_path.replace(".mcap", ".repaired.mcap")
    check_path = repaired_path if os.path.exists(repaired_path) else mcap_path
    
    from mcap.reader import make_reader
    try:
        with open(check_path, "rb") as f:
            reader = make_reader(f)
            for channel in reader.channels.values():
                if channel.topic == '/flight_director/active_waypoint':
                    return True
    except Exception:
        # If index read fails, let try_load_mcap run its repair later
        return True
    return False


def segment_flight_mcap(flight_path, mode="auto", padding_sec=2.0):
    """
    Parses the MCAP inside flight_path, detects all sweeping passes using
    either state-events or kinematics, and slices them into standalone MCAP files.
    """
    folder_name = os.path.basename(flight_path)

    # ── Gate 1: approved date cutoff ──────────────────────────────────────────
    if not _is_approved(folder_name):
        print(f"⏭️  Skipping {folder_name!r}: before approved cutoff ({APPROVED_CUTOFF}).")
        return

    # ── Gate 2: already segmented? ────────────────────────────────────────────
    existing_passes = glob.glob(os.path.join(flight_path, "*-pass*.mcap"))
    if existing_passes:
        print(f"✅ Already segmented ({len(existing_passes)} pass file(s) found): {folder_name}")
        return

    # ── Gate 3: Choose slicing strategy dynamically ───────────────────────────
    has_event_topic = has_active_waypoint_topic(flight_path)
    
    selected_mode = mode
    if mode == "auto":
        selected_mode = "event" if has_event_topic else "legacy"

    print(f"\n=============================================")
    print(f"⚡ PROCESSING {folder_name.upper()}")
    print(f"   Selected Slicing Mode: {selected_mode.upper()} "
          f"(Event topic present: {has_event_topic})")
    print(f"=============================================")

    # Load topic data (will auto-repair if corrupted)
    try:
        topic_data, bag_start_ns, source_mcap_path = try_load_mcap(flight_path)
    except Exception as e:
        print(f"❌ [ERROR] Failed to load/repair MCAP for {folder_name}: {e}")
        return

    complete_passes = []  # List of tuples: (pass_label/index, {'start_ns': t, 'end_ns': t})

    # ──────────────────────────────────────────────────────────────────────────
    # MODE A: Event-Driven Slicing Logic
    # ──────────────────────────────────────────────────────────────────────────
    if selected_mode == "event":
        waypoint_topic = '/flight_director/active_waypoint'
        if waypoint_topic not in topic_data:
            print(f"⚠️  Skipping event mode: no active waypoint event logs found.")
            return

        events = []
        for msg in topic_data[waypoint_topic]:
            events.append((msg.log_time_ns, msg.ros_msg.data))
        events.sort(key=lambda x: x[0])

        if not events:
            print(f"⚠️  No events recorded on {waypoint_topic} in {folder_name!r}.")
            return

        passes = {}  # loop_idx -> { 'start_ns': timestamp, 'end_ns': timestamp }
        for t_ns, payload in events:
            t_rel = (t_ns - bag_start_ns) / 1e9
            print(f"   ⏱️  [{t_rel:6.2f}s] -> Event: {payload}")
            
            match = re.match(r'LOOP_(\d+)_(.+)', payload)
            if match:
                loop_idx = int(match.group(1))
                label = match.group(2)
                
                if label == "EXP_END_POINT":
                    passes[loop_idx] = {'start_ns': t_ns, 'end_ns': None}
                elif label in ["OTHER", "EXP_START_POINT"] and loop_idx in passes:
                    if passes[loop_idx]['end_ns'] is None:
                        passes[loop_idx]['end_ns'] = t_ns
            elif payload in ["FINISHED", "LANDING"]:
                for loop_idx, p in passes.items():
                    if p['end_ns'] is None:
                        p['end_ns'] = t_ns

        # Handle unclosed sweeps using the final message time
        last_msg_time_ns = max(m.log_time_ns for msgs in topic_data.values() for m in msgs) if topic_data else None
        for loop_idx, p in passes.items():
            if p['end_ns'] is None and last_msg_time_ns:
                p['end_ns'] = last_msg_time_ns

        for loop_idx in sorted(passes.keys()):
            p = passes[loop_idx]
            if p['start_ns'] is not None and p['end_ns'] is not None:
                complete_passes.append((f"Pass-{loop_idx:02d}", p))

    # ──────────────────────────────────────────────────────────────────────────
    # MODE B: Legacy Coordinate-Based Slicing Logic
    # ──────────────────────────────────────────────────────────────────────────
    else:
        # Load metadata configurations
        drone_tracker_name, system_config = load_drone_metadata(project_root)
        primary_body  = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "primary"),  {})
        obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})
        cage_radius   = primary_body.get("cage_diameter_m", 0.358) / 2.0
        column_radius = obstacle_body.get("diameter_m", 0.09) / 2.0

        try:
            dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
        except Exception as e:
            print(f"❌ [ERROR] Failed to build legacy coordinate dataframes: {e}")
            return

        df_mocap    = dfs['mocap']
        df_setpoint = dfs.get('setpoint', pd.DataFrame())
        df_column   = dfs.get('column', pd.DataFrame())
        arming_time = dfs['arming_time']
        dynamic_waypoints = dfs.get('dynamic_waypoints', [])

        if df_mocap.empty:
            print("❌ [ERROR] MoCap dataframe is empty. Cannot segment legacy passes.")
            return

        df_mocap = compute_velocity(df_mocap)

        takeoff_mask = df_mocap['z'] > 0.15
        takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else arming_time + 2.0

        if df_column is not None and not df_column.empty:
            col_x, col_y = df_column['x'].head(50).mean(), df_column['y'].head(50).mean()
        else:
            col_x, col_y = 0.408, 0.358

        wp_events_list = find_waypoint_events(
            df_mocap, df_setpoint, takeoff_time, label=folder_name,
            column_x=col_x, column_y=col_y,
            column_radius=column_radius, cage_radius=cage_radius,
            return_all=True, dynamic_waypoints=dynamic_waypoints
        )

        if not wp_events_list:
            print("⚠️  No coordinate-based sweeping passes detected.")
            return

        # Map complete passes (must contain WP2/WP1 and WP3)
        legacy_passes = [
            (idx, evs) for idx, evs in enumerate(wp_events_list)
            if ('WP2' in evs or 'WP1' in evs) and 'WP3' in evs
        ]

        for slice_idx, (orig_idx, wp_events) in enumerate(legacy_passes):
            wp2_t = wp_events.get('WP2') or wp_events.get('WP1')
            wp3_t = wp_events.get('WP3')
            
            t_start_ns = bag_start_ns + int((wp2_t) * 1e9)
            t_end_ns   = bag_start_ns + int((wp3_t) * 1e9)
            complete_passes.append((f"Pass-{slice_idx+1:02d}", {'start_ns': t_start_ns, 'end_ns': t_end_ns}))

    # ──────────────────────────────────────────────────────────────────────────
    # Message Slicing Implementation
    # ──────────────────────────────────────────────────────────────────────────
    if not complete_passes:
        print(f"⚠️  No complete passes detected — no slice files created.")
        return

    print(f"🎯 Slicing {len(complete_passes)} complete pass(es).")
    base_name = os.path.basename(source_mcap_path).replace(".repaired.mcap", "").replace(".mcap", "")
    padding_ns = int(padding_sec * 1e9)

    for idx, (label, p) in enumerate(complete_passes):
        t_start_ns = p['start_ns'] - padding_ns
        t_end_ns   = p['end_ns'] + padding_ns

        t_start_rel = max(0.0, (t_start_ns - bag_start_ns) / 1e9)
        t_end_rel   = (t_end_ns - bag_start_ns) / 1e9

        output_mcap_name = f"{base_name}-pass{idx+1:02d}.mcap"
        output_mcap_path = os.path.join(flight_path, output_mcap_name)

        print(f"🎬 Slicing {label} as Pass {idx+1:02d}: {t_start_rel:.2f}s → {t_end_rel:.2f}s")
        print(f"   💾 Output: {output_mcap_name}")

        msg_count = 0
        with open(output_mcap_path, "wb") as f_out:
            writer = Writer(f_out)
            schema_map = {}
            for msg in read_ros2_messages(source_mcap_path):
                if t_start_ns <= msg.log_time_ns <= t_end_ns:
                    if msg.schema.name not in schema_map:
                        schema_map[msg.schema.name] = writer.register_msgdef(
                            datatype=msg.schema.name,
                            msgdef_text=msg.schema.data.decode()
                        )
                    writer.write_message(
                        topic=msg.channel.topic,
                        schema=schema_map[msg.schema.name],
                        message=msg.ros_msg,
                        log_time=msg.log_time_ns,
                        publish_time=msg.publish_time_ns,
                        sequence=msg.sequence_count
                    )
                    msg_count += 1
            writer.finish()

        print(f"   ✅ Wrote {msg_count} messages → {output_mcap_name}")
    print("=============================================\n")


def segment_all_flights(flights_dir, mode="auto", padding_sec=2.0):
    """Runs the segmenter over all approved flight folders in flights_dir."""
    folders = sorted([
        os.path.join(flights_dir, d)
        for d in os.listdir(flights_dir)
        if os.path.isdir(os.path.join(flights_dir, d))
    ])

    print(f"🔍 Found {len(folders)} flight folder(s) to check.\n")
    for folder in folders:
        segment_flight_mcap(folder, mode=mode, padding_sec=padding_sec)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Unified Telemetry MCAP Pass Segmenter")
    parser.add_argument("flight_path", nargs="?", default=None, 
                        help="Specific flight folder to segment. If omitted, segments all approved flights.")
    parser.add_argument("--mode", choices=["auto", "event", "legacy"], default="auto", 
                        help="Slicing mode. auto: scans for waypoint transitions, falls back to coordinates.")
    parser.add_argument("--padding", type=float, default=2.0, 
                        help="Temporal padding in seconds around the detected sweep pass window (default: 2.0s).")

    args = parser.parse_args()

    # If running with python -m, current_dir is inside dev_logs/analysis/database/
    flights_root = os.path.abspath(os.path.join(current_dir, "..", "..", "flights"))

    if args.flight_path:
        target = args.flight_path
        if os.path.isdir(target):
            segment_flight_mcap(target, mode=args.mode, padding_sec=args.padding)
        else:
            print(f"Error: Target directory not found: {target}")
            sys.exit(1)
    else:
        segment_all_flights(flights_root, mode=args.mode, padding_sec=args.padding)
