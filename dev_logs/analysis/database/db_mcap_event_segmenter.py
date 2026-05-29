#!/usr/bin/env python3
"""
MCAP Event-Based Pass Segmenter
================================
Post-flight utility that physically slices a full raw flight recording into
separate, self-contained pass-level .mcap files based strictly on 
`/flight_director/active_waypoint` ROS 2 state transition event messages.

Rules:
- Only processes flights recorded ON OR AFTER the approved cutoff date
  (flight_20260524-1904, i.e. 2026-05-24 19:04).
- Automatically repairs corrupted MCAP files before slicing (re-writes
  a clean copy from recovered records; original is preserved).
- Uses the `/flight_director/active_waypoint` topic to slice passes.
- Padded by ±2s around the detected EXP_END_POINT transition window.

Usage:
    python3 mcap_event_segmenter.py                          # process all approved flights
    python3 mcap_event_segmenter.py <path_to_flight_folder>  # process one specific folder
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

from dev_logs.analysis.database.db_loader import load_mcap

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
        # If index read fails (corrupted file), let the try_load_mcap flow run its repair
        return True
    return False


def segment_flight_mcap(flight_path, padding_sec=2.0):
    """
    Parses the MCAP inside flight_path, detects all sweeping passes using
    published `/flight_director/active_waypoint` transition events,
    and slices the original messages into standalone pass-specific MCAP files.
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

    # ── Gate 3: contains active waypoint topic? (Ultra-fast check) ────────────
    if not has_active_waypoint_topic(flight_path):
        print(f"⏭️  Skipping {folder_name!r}: no active waypoint topic found in channels list.")
        return

    # Load topic data (will auto-repair if corrupted)
    try:
        topic_data, bag_start_ns, source_mcap_path = try_load_mcap(flight_path)
    except Exception as e:
        print(f"⚠️  Skipping {folder_name!r}: failed to load MCAP: {e}")
        return

    waypoint_topic = '/flight_director/active_waypoint'
    if waypoint_topic not in topic_data:
        print(f"⏭️  Skipping {folder_name!r}: no active waypoint event logs found.")
        return

    # Extract all event transitions
    events = []
    for msg in topic_data[waypoint_topic]:
        events.append((msg.log_time_ns, msg.ros_msg.data))
    
    events.sort(key=lambda x: x[0])

    if not events:
        print(f"⚠️  No events recorded on {waypoint_topic} in {folder_name!r}.")
        return

    print(f"\n=============================================")
    print(f"⚡ PROCESSING EVENTS FOR: {folder_name}")
    print(f"=============================================")

    # Detect sweep passes
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

    # Handle unclosed sweeps (e.g. abrupt stop of bag recording) using the final msg time
    last_msg_time_ns = max(m.log_time_ns for msgs in topic_data.values() for m in msgs) if topic_data else None
    for loop_idx, p in passes.items():
        if p['end_ns'] is None and last_msg_time_ns:
            p['end_ns'] = last_msg_time_ns

    complete_passes = []
    for loop_idx in sorted(passes.keys()):
        p = passes[loop_idx]
        if p['start_ns'] is not None and p['end_ns'] is not None:
            complete_passes.append((loop_idx, p))

    if not complete_passes:
        print(f"⚠️  No valid sweep loops detected in {folder_name!r}.")
        return

    print(f"🎯 Found {len(complete_passes)} sweep pass(es) to slice.")
    base_name = os.path.basename(source_mcap_path).replace(".repaired.mcap", "").replace(".mcap", "")

    padding_ns = int(padding_sec * 1e9)

    for slice_idx, (loop_idx, p) in enumerate(complete_passes):
        t_start_ns = p['start_ns'] - padding_ns
        t_end_ns   = p['end_ns'] + padding_ns

        t_start_rel = max(0.0, (t_start_ns - bag_start_ns) / 1e9)
        t_end_rel   = (t_end_ns - bag_start_ns) / 1e9

        output_mcap_name = f"{base_name}-pass{slice_idx+1:02d}.mcap"
        output_mcap_path = os.path.join(flight_path, output_mcap_name)

        print(f"🎬 Slicing Loop {loop_idx} as Pass {slice_idx+1:02d}: {t_start_rel:.2f}s → {t_end_rel:.2f}s")
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


def segment_all_flights(flights_dir):
    """Runs the event segmenter over all flight folders in flights_dir."""
    folders = sorted([
        os.path.join(flights_dir, d)
        for d in os.listdir(flights_dir)
        if os.path.isdir(os.path.join(flights_dir, d))
    ])

    print(f"🔍 Found {len(folders)} flight folder(s) to check.\n")
    for folder in folders:
        segment_flight_mcap(folder)


if __name__ == "__main__":
    flights_root = os.path.join(os.path.dirname(os.path.dirname(current_dir)), "flights")
    
    if len(sys.argv) > 1:
        target = sys.argv[1]
        if os.path.isdir(target):
            segment_flight_mcap(target)
        else:
            print(f"Error: Target directory not found: {target}")
            sys.exit(1)
    else:
        segment_all_flights(flights_root)
