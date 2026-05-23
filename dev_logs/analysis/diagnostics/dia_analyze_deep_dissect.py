#!/usr/bin/env python3
"""
Deep dissection of the flight telemetry bag and ULog.
Produces time-series tables comparable to the flight envelope audits.
"""
import numpy as np
import datetime
import glob
import os
import sys
from mcap_ros2.reader import read_ros2_messages
from pyulog import ULog

def run_deep_dissection(bag_path=None, ulog_path=None):
    """
    Executes a high-frequency telemetry dissection on a ROS2 bag and PX4 ULog.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    analysis_dir = os.path.dirname(script_dir)
    dev_logs_dir = os.path.dirname(analysis_dir)
    project_root = os.path.dirname(dev_logs_dir)
    
    # 1. Resolve Bag Path (fallback to latest flight bag if not specified)
    if bag_path is None:
        flights_dir = os.path.join(dev_logs_dir, "flights")
        flight_folders = glob.glob(os.path.join(flights_dir, "flight_*"))
        flight_folders = [d for d in flight_folders if os.path.isdir(d) and "px4_sd_logs" not in d]
        if not flight_folders:
            print("❌ Error: No flight directories found to analyze.")
            return
        latest_folder = max(flight_folders, key=os.path.getmtime)
        mcap_files = glob.glob(os.path.join(latest_folder, "*.mcap"))
        if not mcap_files:
            print(f"❌ Error: No .mcap files found in {latest_folder}")
            return
        bag_path = mcap_files[0]
        
    # 2. Resolve ULog Path (fallback to latest ULog if not specified)
    if ulog_path is None:
        ulogs_dir = os.path.join(dev_logs_dir, "flights", "px4_sd_logs")
        ulog_files = glob.glob(os.path.join(ulogs_dir, "**", "*.ulg"), recursive=True)
        if ulog_files:
            ulog_path = max(ulog_files, key=os.path.getmtime)
            
    print("=" * 65)
    print("🔬 TELEMETRY DEEP DISSECTION ACTIVE")
    print("=" * 65)
    print(f"📂 ROS2 Bag: {bag_path}")
    if ulog_path:
        print(f"💾 PX4 ULog: {ulog_path}")
    print("=" * 65 + "\n")

    # ─── Load Bag ───
    print("Loading bag...")
    topic_data = {}
    try:
        for msg in read_ros2_messages(bag_path):
            t = msg.channel.topic
            if t not in topic_data:
                topic_data[t] = []
            topic_data[t].append(msg)
    except Exception as e:
        print(f"❌ Error reading bag file: {e}")
        return
        
    total_msgs = sum(len(v) for v in topic_data.values())
    if total_msgs == 0:
        print("❌ Error: Bag is empty.")
        return
        
    print(f"Loaded {total_msgs} messages.\n")

    # ─── Normalize all times relative to bag start ───
    bag_start = min(m.log_time_ns for msgs in topic_data.values() for m in msgs) * 1e-9
    def rel(ns): return ns * 1e-9 - bag_start

    # ─────────────────────────────────────────────
    # A. VVO GAP DISTRIBUTION
    # ─────────────────────────────────────────────
    print("=" * 65)
    print("A. VVO GAP DISTRIBUTION — Was the 59ms gap an outlier or systemic?")
    print("=" * 65)

    vvo_msgs = topic_data.get("/fmu/in/vehicle_visual_odometry", [])
    if len(vvo_msgs) > 1:
        vvo_times = np.array(sorted([rel(m.log_time_ns) for m in vvo_msgs]))
        vvo_gaps_ms = np.diff(vvo_times) * 1000

        buckets = [(0,10),(10,20),(20,30),(30,40),(40,50),(50,60),(60,200)]
        print(f"\n  Total VVO messages: {len(vvo_times)}, Duration: {vvo_times[-1]:.1f}s")
        print(f"  Mean gap: {np.mean(vvo_gaps_ms):.1f}ms  Median: {np.median(vvo_gaps_ms):.1f}ms")
        print(f"\n  Gap Distribution:")
        print(f"  {'Range (ms)':<18} {'Count':>7}  {'%':>7}  {'Interpretation'}")
        print(f"  {'-'*60}")
        for lo, hi in buckets:
            mask = (vvo_gaps_ms >= lo) & (vvo_gaps_ms < hi)
            n = np.sum(mask)
            pct = 100.0 * n / len(vvo_gaps_ms)
            flag = ""
            if lo >= 33: flag = "⚠️ Below 30Hz threshold"
            if lo >= 50: flag = "🔴 Below 20Hz"
            print(f"  {lo:>5}–{hi:<10} {n:>7}  {pct:>6.1f}%  {flag}")

        # Show when the big gaps occurred
        big_gap_idx = np.where(vvo_gaps_ms > 33)[0]
        if len(big_gap_idx) > 0:
            print(f"\n  All gaps > 33ms ({len(big_gap_idx)} total):")
            print(f"  {'t_start(s)':>12}  {'t_end(s)':>10}  {'gap(ms)':>9}  Relative to test start")
            print(f"  {'-'*55}")
            for idx in big_gap_idx[:30]:
                t0 = vvo_times[idx]
                t1 = vvo_times[idx+1]
                g = vvo_gaps_ms[idx]
                print(f"  {t0:>12.3f}  {t1:>10.3f}  {g:>9.1f}ms")
    else:
        print("\n  No vehicle visual odometry messages captured to compute gaps.")

    # ─────────────────────────────────────────────
    # B. DRONE POSITION OVER TIME (Was it in the Mocap volume?)
    # ─────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("B. DRONE POSITION OVER TIME — Did it leave the Mocap volume?")
    print("=" * 65)

    poses_msgs = topic_data.get("/poses", [])
    pos_times, pos_x, pos_y, pos_z = [], [], [], []
    for m in poses_msgs:
        for p in m.ros_msg.poses:
            p_name = p.name.lower()
            if "column" in p_name or "origin" in p_name:
                continue
            if any(k in p_name for k in ["jake_drone", "drone_frame", "jake", "drone", "arrow", "frame"]):
                pos_times.append(rel(m.log_time_ns))
                pos_x.append(p.pose.position.x)
                pos_y.append(p.pose.position.y)
                pos_z.append(p.pose.position.z)
                break

    if len(pos_times) > 0:
        pos_times = np.array(pos_times)
        pos_x = np.array(pos_x)
        pos_y = np.array(pos_y)
        pos_z = np.array(pos_z)

        print(f"\n  Tracking body samples: {len(pos_times)}")
        print(f"\n  {'t(s)':>6}  {'X(m)':>8}  {'Y(m)':>8}  {'Z(m)':>8}  Status")
        print(f"  {'-'*55}")
        for t_sample in np.arange(0, pos_times[-1], 5.0):
            idx = np.searchsorted(pos_times, t_sample)
            if idx >= len(pos_times): break
            x, y, z = pos_x[idx], pos_y[idx], pos_z[idx]
            r = np.sqrt(x**2 + y**2)
            status = "✅ In volume" if r < 4.0 and abs(z) < 3.0 else "⚠️ Possibly outside"
            print(f"  {t_sample:>6.1f}  {x:>8.3f}  {y:>8.3f}  {z:>8.3f}  {status}")
    else:
        print("\n  No drone poses captured in /poses.")

    # ─────────────────────────────────────────────
    # C. CORRELATE REJECTIONS WITH MOTION (from ULog)
    # ─────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("C. FUSION REJECTION vs MOTION — When was EV rejected?")
    print("=" * 65)

    if ulog_path and os.path.exists(ulog_path):
        try:
            ulog = ULog(ulog_path, message_name_filter_list=[
                "estimator_status_flags", "vehicle_local_position", "estimator_innovations"
            ])
            avail = [d.name for d in ulog.data_list]
            print(f"\n  ULog available datasets: {avail}")

            if "estimator_status_flags" in avail:
                flags = ulog.get_dataset("estimator_status_flags")
                t_flags = np.array(flags.data["timestamp"]) / 1e6  # convert us → s
                ev_pos  = np.array(flags.data.get("cs_ev_pos", []))
                ev_hgt  = np.array(flags.data.get("cs_ev_hgt", []))

                if len(ev_pos) > 0:
                    print(f"\n  Total flag samples: {len(t_flags)}")
                    print(f"  EV Pos FUSED:    {100*np.mean(ev_pos>0):.1f}%")
                    print(f"  EV Height FUSED: {100*np.mean(ev_hgt>0):.1f}%")

                    # Window analysis — 10-second buckets
                    print(f"\n  Fusion rate in 10-second windows (relative to ULog start):")
                    print(f"  {'Window':>15}  {'EV Pos%':>8}  {'EV Hgt%':>8}  {'Samples':>8}  Assessment")
                    print(f"  {'-'*65}")
                    t_rel = t_flags - t_flags[0]  # make relative
                    for w_start in np.arange(0, t_rel[-1], 10.0):
                        w_end = w_start + 10.0
                        mask = (t_rel >= w_start) & (t_rel < w_end)
                        if np.sum(mask) < 5: continue
                        pos_fused = 100.0 * np.mean(ev_pos[mask] > 0)
                        hgt_fused = 100.0 * np.mean(ev_hgt[mask] > 0)
                        n = np.sum(mask)
                        flag = "✅" if pos_fused > 85 else "⚠️" if pos_fused > 50 else "🔴"
                        print(f"  {w_start:>6.0f}–{w_end:<6.0f}s  {pos_fused:>7.0f}%  {hgt_fused:>7.0f}%  {n:>8}  {flag}")

            if "estimator_innovations" in avail:
                innov = ulog.get_dataset("estimator_innovations")
                innov_keys = list(innov.data.keys())
                print(f"\n  Innovation fields available: {[k for k in innov_keys if 'ev' in k.lower() or 'innov' in k.lower()][:10]}")
                
                # EV position innovations
                for key in ["ev_hpos_innov[0]", "ev_vpos_innov", "ev_hpos_innov_var[0]"]:
                    val = innov.data.get(key, None)
                    if val is not None:
                        arr = np.abs(np.array(val))
                        print(f"  {key}: mean={np.mean(arr):.3f}, max={np.max(arr):.3f}, "
                              f">0.3m: {np.sum(arr>0.3)} samples ({100*np.mean(arr>0.3):.0f}%)")

        except Exception as e:
            print(f"  ULog error: {e}")
    else:
        print("\n  No PX4 ULog provided or found. Skipping fusion rejection correlation.")

    # ─────────────────────────────────────────────
    # D. VVO RATE vs POSE RATE WINDOW-BY-WINDOW
    # ─────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("D. MOCAP vs VVO RATE — 5-Second Windows")
    print("=" * 65)

    if len(pos_times) > 0 and len(vvo_msgs) > 0:
        pose_times_rel = np.array(sorted([rel(m.log_time_ns) for m in poses_msgs]))
        vvo_times_rel  = vvo_times
        total_dur = max(pose_times_rel[-1], vvo_times_rel[-1])

        print(f"\n  {'Window':>12}  {'Poses':>6}  {'PoseHz':>8}  {'MaxGap':>9}  {'VVO':>6}  {'VVOHz':>8}  {'VVOMaxGap':>10}")
        print(f"  {'-'*75}")
        for w in np.arange(0, total_dur, 5.0):
            p_mask = (pose_times_rel >= w) & (pose_times_rel < w+5)
            v_mask = (vvo_times_rel  >= w) & (vvo_times_rel  < w+5)
            n_p = np.sum(p_mask)
            n_v = np.sum(v_mask)
            hz_p = n_p / 5.0
            hz_v = n_v / 5.0
            p_times = pose_times_rel[p_mask]
            v_times = vvo_times_rel[v_mask]
            max_gap_p = np.max(np.diff(p_times))*1000 if len(p_times) > 1 else 0
            max_gap_v = np.max(np.diff(v_times))*1000 if len(v_times) > 1 else 0
            vflag = "⚠️" if max_gap_v > 33 else "✅"
            print(f"  {w:>5.0f}–{w+5:<5.0f}s  {n_p:>6}  {hz_p:>7.1f}Hz  {max_gap_p:>7.1f}ms"
                  f"  {n_v:>6}  {hz_v:>7.1f}Hz  {max_gap_v:>8.1f}ms {vflag}")
    else:
        print("\n  Telemetry data stream too short to slice window frequencies.")

    print("\n=== DEEP DISSECTION COMPLETE ===")

if __name__ == '__main__':
    run_deep_dissection()
