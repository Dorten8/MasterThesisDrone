#!/usr/bin/env python3
"""
ULog Copier and Matcher (Optimized Sliding-Window Sync)
======================================================
Matches flight MCAP bags with raw PX4 .ulg files globally across SD card log directories
within a ±2-day sliding window of the flight folder's date, avoiding scanning unrelated large files.
"""

import os
import glob
import shutil
import sys
import datetime
import pyulog
from mcap_ros2.reader import read_ros2_messages

flights_dir = "/home/dorten/MasterThesisDrone/dev_logs/flights"
px4_log_root = "/media/dorten/PX4_32G/log"
log_file_path = "/home/dorten/MasterThesisDrone/dev_logs/analysis/database/ulog_matching_log.txt"

def log_msg(msg, log_file):
    print(msg)
    log_file.write(msg + "\n")

def get_mcap_comp_time_range(mcap_files):
    """Finds companion epoch seconds range from MCAP files."""
    sorted_mcaps = sorted(mcap_files, key=lambda f: (
        "repaired" not in os.path.basename(f),
        "-pass" in os.path.basename(f)
    ))
    
    for mf in sorted_mcaps:
        comp_times = []
        try:
            for msg in read_ros2_messages(mf):
                comp_times.append(msg.log_time_ns * 1e-9)
            if comp_times:
                return min(comp_times), max(comp_times)
        except Exception:
            continue
    return None

def get_ulog_comp_time_range(ulg_path):
    """Extracts start and end companion clock timestamps (in seconds)
    from the ULog file's timesync_status.remote_timestamp field.
    """
    try:
        ul = pyulog.ULog(ulg_path, message_name_filter_list=["timesync_status"])
        dataset = ul.get_dataset("timesync_status")
        if len(dataset.data["remote_timestamp"]) > 0:
            remote_ts = dataset.data["remote_timestamp"]
            return remote_ts[0] * 1e-6, remote_ts[-1] * 1e-6
    except Exception:
        pass
    return None

def get_search_dates(date_str, window_days=2):
    """Given a YYYY-MM-DD date string, returns a list of YYYY-MM-DD strings in [date-window, date+window]."""
    try:
        base_date = datetime.datetime.strptime(date_str, "%Y-%m-%d").date()
        dates = []
        for d in range(-window_days, window_days + 1):
            dates.append((base_date + datetime.timedelta(days=d)).strftime("%Y-%m-%d"))
        return dates
    except Exception:
        return [date_str]

def main():
    with open(log_file_path, "w") as log_file:
        log_msg("=== ULog Matching Log ===", log_file)
        log_msg(f"Date: {datetime.datetime.now()}", log_file)
        
        if not os.path.exists(px4_log_root):
            log_msg(f"❌ PX4 log directory not found at: {px4_log_root}", log_file)
            return

        flight_folders = sorted(glob.glob(os.path.join(flights_dir, "flight_*")))
        log_msg(f"🔍 Found {len(flight_folders)} flight folders in {flights_dir}.\n", log_file)

        copied_count = 0
        skipped_count = 0
        failed_count = 0
        unmatched_folders = []

        # Shared cache for ULog time ranges to prevent re-reading files across flight folders
        ulg_time_range_cache = {}

        # Step 2: Match each flight folder
        for ff in flight_folders:
            folder_name = os.path.basename(ff)
            
            # Skip pre-admissible parameter-tweaking flights on May 24
            if folder_name.startswith("flight_20260524-"):
                time_part = folder_name.split("-")[1].split("_")[0]
                if int(time_part) < 1904:
                    log_msg(f"skip: Skipping pre-admissible flight: {folder_name}", log_file)
                    skipped_count += 1
                    continue
                
            parts = folder_name.split("_")
            if len(parts) < 2:
                continue
            date_part = parts[1].split("-")[0] # YYYYMMDD
            if len(date_part) != 8:
                continue
            
            year = date_part[0:4]
            month = date_part[4:6]
            day = date_part[6:8]
            flight_date_str = f"{year}-{month}-{day}"

            mcap_files = glob.glob(os.path.join(ff, "*.mcap"))
            if not mcap_files:
                log_msg(f"⚠️ No MCAP files found in {folder_name}", log_file)
                failed_count += 1
                continue

            mcap_range = get_mcap_comp_time_range(mcap_files)
            if not mcap_range:
                log_msg(f"❌ Primary timesync check failed: Could not read MCAP times in {folder_name}", log_file)
                failed_count += 1
                continue
                
            mcap_comp_start, mcap_comp_end = mcap_range
            matched_ulg = None

            # Get search dates within +/- 2 days window of flight date
            search_dates = get_search_dates(flight_date_str, window_days=2)
            
            # Collect and scan all .ulg files in these directories
            candidate_ulgs = []
            for ds in search_dates:
                ulg_dir = os.path.join(px4_log_root, ds)
                if os.path.exists(ulg_dir):
                    candidate_ulgs.extend(glob.glob(os.path.join(ulg_dir, "*.ulg")))
            
            # Ensure we read timesync ranges for candidates and search for a match
            for uf in sorted(candidate_ulgs):
                if uf not in ulg_time_range_cache:
                    # Log scanning only when we first access a file (helps debugging performance)
                    size_mb = os.path.getsize(uf) / (1024 * 1024)
                    print(f"   [Scan] {os.path.basename(os.path.dirname(uf))}/{os.path.basename(uf)} ({size_mb:.1f} MB)...")
                    ulg_range = get_ulog_comp_time_range(uf)
                    ulg_time_range_cache[uf] = ulg_range
                
                ulg_range = ulg_time_range_cache[uf]
                if not ulg_range:
                    continue
                ulg_comp_start, ulg_comp_end = ulg_range
                
                # Check containment in companion clock domain (with a tight 5s buffer)
                if ulg_comp_start <= mcap_comp_start + 5.0 and mcap_comp_end <= ulg_comp_end + 5.0:
                    matched_ulg = uf
                    break

            if not matched_ulg:
                log_msg(f"❌ Primary timesync check failed: No matching ULog found for {folder_name}", log_file)
                log_msg(f"   (MCAP span: {mcap_comp_start:.1f} to {mcap_comp_end:.1f}, searched window around {flight_date_str})", log_file)
                unmatched_folders.append(folder_name)
                failed_count += 1
                continue

            matched_filename = os.path.basename(matched_ulg)
            dest_path = os.path.join(ff, matched_filename)
            
            existing_ulogs = glob.glob(os.path.join(ff, "*.ulg"))
            if existing_ulogs:
                existing_filename = os.path.basename(existing_ulogs[0])
                if existing_filename == matched_filename:
                    log_msg(f"✅ ULog already present and correct in {folder_name}: {matched_filename}", log_file)
                    skipped_count += 1
                    continue
                else:
                    log_msg(f"🧹 Deleting mismatched ULog {existing_filename} from {folder_name}", log_file)
                    os.remove(existing_ulogs[0])

            log_msg(f"⚡ Matched {folder_name} -> {matched_filename} (from folder: {os.path.basename(os.path.dirname(matched_ulg))})", log_file)
            shutil.copy2(matched_ulg, dest_path)
            copied_count += 1

        log_msg(f"\n📊 ULog Copying Summary:", log_file)
        log_msg(f"   Matched & Copied: {copied_count}", log_file)
        log_msg(f"   Already Present:  {skipped_count}", log_file)
        log_msg(f"   Failed to Match:  {failed_count}", log_file)
        log_msg(f"   Total Folders:    {len(flight_folders)}", log_file)
        
        if unmatched_folders:
            log_msg("\n=== Summary of Flight Folders Missing Corresponding ULogs ===", log_file)
            for idx, uf_name in enumerate(unmatched_folders):
                log_msg(f"{idx+1}. {uf_name}", log_file)

if __name__ == "__main__":
    main()
