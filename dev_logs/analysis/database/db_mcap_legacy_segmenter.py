#!/usr/bin/env python3
"""
MCAP Pass Segmenter
===================
Post-flight utility that physically slices a full raw flight recording into
separate, self-contained pass-level .mcap files.

Rules:
- Only processes flights recorded ON OR AFTER the approved cutoff date
  (flight_20260524-1904, i.e. 2026-05-24 19:04).
- Automatically repairs corrupted MCAP files before slicing (re-writes
  a clean copy from recovered records; original is preserved).
- Only creates a slice file if a COMPLETE pass is detected (WP2 + WP3 both found).
- Skips flights that have already been sliced (a -passXX.mcap file already exists).
- Padded by ±2s around the detected WP2→WP3 sweep window.

Usage:
    python3 mcap_segmenter.py                          # process all approved flights
    python3 mcap_segmenter.py <path_to_flight_folder>  # process one specific folder
"""
import os
import sys
import glob
import re
import pandas as pd
from mcap_ros2.reader import read_ros2_messages
from mcap_ros2.writer import Writer

# Resolve package directory and inject into python path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from dev_logs.analysis.database.db_loader import load_drone_metadata, load_mcap, build_dataframes
from dev_logs.analysis.kinematics.kin_calculator import compute_velocity, find_waypoint_events

# ── Approved flight cutoff ────────────────────────────────────────────────────
# Only flights with a timestamp >= this value (YYYYMMDD-HHMM) are eligible.
APPROVED_CUTOFF = "20260524-1904"


def _flight_timestamp(folder_name):
    """Extracts YYYYMMDD-HHMM from a flight folder name, or None if not parseable."""
    m = re.search(r'flight_(\d{8}-\d{4})', folder_name)
    return m.group(1) if m else None


def _is_approved(folder_name):
    """Returns True if the flight is at or after the approved cutoff."""
    ts = _flight_timestamp(folder_name)
    if ts is None:
        return False
    return ts >= APPROVED_CUTOFF


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
    # Find the primary file (not a pass slice, not a repaired copy)
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
        from mcap_ros2.reader import read_ros2_messages as _rr
        ROS2DecoderFactory = None

    topic_data = {}
    with open(repaired, "rb") as f:
        reader = make_reader(f, decoder_factories=[ROS2DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages():
            t = channel.topic
            if t not in topic_data:
                topic_data[t] = []
            # Wrap in a lightweight namespace so build_dataframes can access .ros_msg and .log_time_ns
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


def segment_flight_mcap(flight_path, padding_sec=2.0):
    """
    Parses the MCAP inside flight_path, detects all complete sweeping passes,
    and slices the original messages into standalone pass-specific MCAP files.
    """
    folder_name = os.path.basename(flight_path)

    # ── Gate: approved date cutoff ────────────────────────────────────────────
    if not _is_approved(folder_name):
        print(f"⏭️  Skipping {folder_name!r}: before approved cutoff ({APPROVED_CUTOFF}).")
        return

    # ── Gate: already sliced? ─────────────────────────────────────────────────
    existing_passes = glob.glob(os.path.join(flight_path, "*-pass*.mcap"))
    if existing_passes:
        print(f"✅ Already segmented ({len(existing_passes)} pass file(s) found): {folder_name}")
        return

    print(f"\n⚡ Starting pass segmenter for: {folder_name}")

    project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

    drone_tracker_name, system_config = load_drone_metadata(project_root)
    primary_body  = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "primary"),  {})
    obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})
    cage_radius   = primary_body.get("cage_diameter_m", 0.358) / 2.0
    column_radius = obstacle_body.get("diameter_m", 0.09) / 2.0

    try:
        topic_data, bag_start_ns, source_mcap_path = try_load_mcap(flight_path)
    except Exception as e:
        print(f"❌ [ERROR] Failed to load/repair MCAP: {e}")
        return

    try:
        from dev_logs.analysis.database.db_loader import build_dataframes
        dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
    except Exception as e:
        print(f"❌ [ERROR] Failed to build dataframes: {e}")
        return

    df_mocap          = dfs['mocap']
    df_setpoint       = dfs.get('setpoint', pd.DataFrame())
    df_column         = dfs.get('column', pd.DataFrame())
    arming_time       = dfs['arming_time']
    dynamic_waypoints = dfs.get('dynamic_waypoints', [])

    if df_mocap.empty:
        print("❌ [ERROR] MoCap dataframe is empty. Cannot segment passes.")
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
        print("⚠️  No valid sweeping passes detected — no slice files created.")
        return

    # Filter to complete passes only
    complete_passes = [
        (idx, evs) for idx, evs in enumerate(wp_events_list)
        if ('WP2' in evs or 'WP1' in evs) and 'WP3' in evs
    ]

    if not complete_passes:
        print("⚠️  No COMPLETE passes detected (missing WP2 or WP3) — no slice files created.")
        return

    print(f"🎯 {len(complete_passes)} complete pass(es) to slice (out of {len(wp_events_list)} detected).")

    base_name = os.path.basename(source_mcap_path).replace(".repaired.mcap", "").replace(".mcap", "")

    for slice_idx, (orig_idx, wp_events) in enumerate(complete_passes):
        wp2_t = wp_events.get('WP2') or wp_events.get('WP1')
        wp3_t = wp_events.get('WP3')

        t_start_rel = max(0.0, wp2_t - padding_sec)
        t_end_rel   = wp3_t + padding_sec
        t_start_ns  = bag_start_ns + int(t_start_rel * 1e9)
        t_end_ns    = bag_start_ns + int(t_end_rel * 1e9)

        output_mcap_name = f"{base_name}-pass{slice_idx+1:02d}.mcap"
        output_mcap_path = os.path.join(flight_path, output_mcap_name)

        print(f"🎬 Slicing Pass {slice_idx+1:02d}: {t_start_rel:.2f}s → {t_end_rel:.2f}s")
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


def segment_all_flights(flights_dir):
    """Runs the segmenter over all flight folders in flights_dir."""
    folders = sorted([
        os.path.join(flights_dir, d)
        for d in os.listdir(flights_dir)
        if os.path.isdir(os.path.join(flights_dir, d))
    ])
    print(f"🔍 Found {len(folders)} flight folder(s) to check.\n")
    for folder in folders:
        segment_flight_mcap(folder)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        segment_flight_mcap(sys.argv[1])
    else:
        project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
        flights_dir  = os.path.join(project_root, "dev_logs", "flights")
        segment_all_flights(flights_dir)
