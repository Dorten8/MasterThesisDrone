#!/usr/bin/env python3
"""Targeted DB column repopulation — load ONLY IMU from pass MCAPs, compute one metric, UPDATE.

Solves the "30-minute pipeline for one column" problem: when a new metric column
is added to the pipeline that only depends on IMU data and the impact timestamp,
re-running the full pipeline (MoCap kinematics, EKF, ULog parsing, 7 PNGs per
flight) is wasteful.  This tool loads only the `/fmu/out/sensor_combined` topic
from each pass MCAP, finds the impact time via max a_deviation, computes the
metric, and issues a targeted SQL UPDATE.

Usage:
    # Preview what would happen (no DB writes)
    python3 -m dev_logs.analysis.database.repopulate_column imu_delta_v_z --dry-run

    # Actually populate
    python3 -m dev_logs.analysis.database.repopulate_column imu_delta_v_z

Registered columns: imu_delta_v_z, imu_delta_v_x, imu_delta_v_y
"""

import os
import sys
import glob
import re
import sqlite3
import argparse

import numpy as np
import pandas as pd

# ═══════════════════════════════════════════════════════════════════════════════
#  Compute-function registry — add new columns here
# ═══════════════════════════════════════════════════════════════════════════════

COMPUTE_REGISTRY = {}


def register_column(name):
    """Decorator: register a column's compute function."""
    def decorator(func):
        COMPUTE_REGISTRY[name] = func
        return func
    return decorator


@register_column('imu_delta_v_z')
def compute_imu_delta_v_z(df_imu, t_impact):
    r"""ΔV_z = ∫|a_z(t) + g| dt over 400 ms contact window [m/s].

    Shifts Z-acceleration by +g to remove IMU gravity compensation, takes
    absolute value so upward and downward excursions both contribute, then
    integrates via composite trapezoidal rule with dx=0.004 (250 Hz IMU).
    """
    df_contact = df_imu[(df_imu['t'] >= t_impact - 0.05) & (df_imu['t'] <= t_impact + 0.35)]
    if len(df_contact) < 2:
        return 0.0
    return float(np.trapz((df_contact['az'] + 9.81).abs(), dx=0.004))


@register_column('imu_delta_v_x')
def compute_imu_delta_v_x(df_imu, t_impact):
    r"""ΔV_x = ∫|a_x(t)| dt over 400 ms contact window [m/s]."""
    df_contact = df_imu[(df_imu['t'] >= t_impact - 0.05) & (df_imu['t'] <= t_impact + 0.35)]
    if len(df_contact) < 2:
        return 0.0
    return float(np.trapz(df_contact['ax'].abs(), dx=0.004))


@register_column('imu_delta_v_y')
def compute_imu_delta_v_y(df_imu, t_impact):
    r"""ΔV_y = ∫|a_y(t)| dt over 400 ms contact window [m/s]."""
    df_contact = df_imu[(df_imu['t'] >= t_impact - 0.05) & (df_imu['t'] <= t_impact + 0.35)]
    if len(df_contact) < 2:
        return 0.0
    return float(np.trapz(df_contact['ay'].abs(), dx=0.004))


# ═══════════════════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _find_pass_path(pass_name, flights_dir):
    """Reconstruct pass MCAP path from DB pass_name.

    DB primary key format:  "flight_20260524-1757_75°_..._fixed_cage - Pass-01"
    On-disk pass file:      .../flights/flight_20260524-1757_.../flight_..._0-pass01.mcap
    """
    m = re.match(r'(.+?) - Pass-(\d+)$', pass_name)
    if not m:
        return None
    folder_name = m.group(1)
    pass_idx = int(m.group(2))
    folder_dir = os.path.join(flights_dir, folder_name)
    if not os.path.isdir(folder_dir):
        return None

    pattern = os.path.join(folder_dir, f"{folder_name}_0-pass{pass_idx:02d}.mcap")
    matches = glob.glob(pattern)
    if matches:
        return matches[0]

    # Fallback: glob with wildcard (some folders have non-standard naming)
    pattern2 = os.path.join(folder_dir, f"*-pass{pass_idx:02d}.mcap")
    matches2 = glob.glob(pattern2)
    return matches2[0] if matches2 else None


def _load_imu_from_mcap(mcap_path):
    """Load ONLY the IMU (sensor_combined) topic from a pass MCAP file.

    Returns a pd.DataFrame with columns: t, ax, ay, az, gx, gy, gz,
    a_mag, g_mag, a_deviation — identical in structure to build_dataframes().
    """
    from mcap_ros2.reader import read_ros2_messages

    imu_list = []
    bag_start_ns = None

    for msg in read_ros2_messages(mcap_path):
        if bag_start_ns is None:
            bag_start_ns = msg.log_time_ns

        if msg.channel.topic == '/fmu/out/sensor_combined':
            t = (msg.log_time_ns - bag_start_ns) / 1e9
            imu_list.append({
                't': t,
                'ax': msg.ros_msg.accelerometer_m_s2[0],
                'ay': msg.ros_msg.accelerometer_m_s2[1],
                'az': msg.ros_msg.accelerometer_m_s2[2],
                'gx': msg.ros_msg.gyro_rad[0],
                'gy': msg.ros_msg.gyro_rad[1],
                'gz': msg.ros_msg.gyro_rad[2],
            })

    df_imu = pd.DataFrame(imu_list)
    if not df_imu.empty:
        df_imu['a_mag'] = np.sqrt(df_imu['ax']**2 + df_imu['ay']**2 + df_imu['az']**2)
        df_imu['g_mag'] = np.sqrt(df_imu['gx']**2 + df_imu['gy']**2 + df_imu['gz']**2)
        df_imu['a_deviation'] = np.abs(df_imu['a_mag'] - 9.81)

    return df_imu


def _find_impact_time_from_imu(df_imu):
    """Find impact time as time of max a_deviation within the pass window.

    The pass MCAP is pre-sliced to ~10 s around the collision pass, so the
    collision spike dominates the a_deviation signal.  This matches the
    MoCap-based impact detection within a few milliseconds — negligible
    relative to the 400 ms integration window.
    """
    if df_imu.empty:
        return None
    idx_max = df_imu['a_deviation'].idxmax()
    return float(df_imu.loc[idx_max, 't'])


# ═══════════════════════════════════════════════════════════════════════════════
#  Main entry point
# ═══════════════════════════════════════════════════════════════════════════════

def repopulate_column(column_name, dry_run=False, db_path=None, flights_dir=None):
    """Populate a single column for all flights where it is NULL."""
    if column_name not in COMPUTE_REGISTRY:
        print(f"❌ Unknown column: '{column_name}'")
        print(f"   Registered columns: {sorted(COMPUTE_REGISTRY.keys())}")
        return 1

    compute_func = COMPUTE_REGISTRY[column_name]

    # ── Paths ──
    project_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    if db_path is None:
        db_path = os.path.join(
            project_root, 'dev_logs', 'analysis', 'experiments_summary.db')
    if flights_dir is None:
        flights_dir = os.path.join(project_root, 'dev_logs', 'flights')

    # ── Connect & query NULLs ──
    conn = sqlite3.connect(db_path)

    # Ensure column exists (migration may not have been run yet)
    existing = {row[1] for row in conn.execute(
        "PRAGMA table_info(flights_summary)").fetchall()}
    if column_name not in existing:
        print(f"❌ Column '{column_name}' does not exist in flights_summary table.")
        print(f"   Run the DB migration first (db_manager.py auto-migrates on import).")
        conn.close()
        return 1

    df_null = pd.read_sql_query(
        f'SELECT flight_name FROM flights_summary '
        f'WHERE impact_detected = 1 AND "{column_name}" IS NULL '
        f'ORDER BY flight_name',
        conn)

    total_impacts = pd.read_sql_query(
        'SELECT COUNT(*) AS n FROM flights_summary WHERE impact_detected = 1',
        conn)['n'].iloc[0]

    if df_null.empty:
        print(f"✅ All {total_impacts} impact flights already have "
              f"'{column_name}' populated!")
        conn.close()
        return 0

    null_count = len(df_null)
    populated = pd.read_sql_query(
        f'SELECT COUNT(*) AS n FROM flights_summary '
        f'WHERE impact_detected = 1 AND "{column_name}" IS NOT NULL',
        conn)['n'].iloc[0]

    print(f"🔍 {populated}/{total_impacts} already populated, "
          f"{null_count} NULL rows to fill")
    if dry_run:
        print(f"   ⚠️  DRY RUN — no DB writes will be made\n")
    else:
        print()

    updated, skipped, errors = 0, 0, 0

    for i, (_, row) in enumerate(df_null.iterrows(), 1):
        pass_name = row['flight_name']

        # ── Locate pass MCAP ──
        mcap_path = _find_pass_path(pass_name, flights_dir)
        if mcap_path is None:
            print(f"  [{i}/{null_count}] ⚠️  Pass MCAP not found: {pass_name}")
            skipped += 1
            continue

        try:
            # ── Load IMU (only sensor_combined topic) ──
            df_imu = _load_imu_from_mcap(mcap_path)
            if df_imu.empty:
                print(f"  [{i}/{null_count}] ⚠️  No IMU data in MCAP: {pass_name}")
                skipped += 1
                continue

            # ── Find impact time from IMU a_deviation peak ──
            t_impact = _find_impact_time_from_imu(df_imu)
            if t_impact is None:
                print(f"  [{i}/{null_count}] ⚠️  Cannot find impact in IMU: {pass_name}")
                skipped += 1
                continue

            # ── Compute ──
            value = compute_func(df_imu, t_impact)

            # ── Write ──
            if not dry_run:
                conn.execute(
                    f'UPDATE flights_summary SET "{column_name}" = ? '
                    f'WHERE flight_name = ?',
                    (value, pass_name))

            print(f"  [{i}/{null_count}] ✅ {pass_name}: "
                  f"{column_name} = {value:.4f}")
            updated += 1

        except Exception as e:
            print(f"  [{i}/{null_count}] ❌ {pass_name}: {e}")
            errors += 1

    if not dry_run:
        conn.commit()

    # ── Summary ──
    remaining = pd.read_sql_query(
        f'SELECT COUNT(*) AS n FROM flights_summary '
        f'WHERE impact_detected = 1 AND "{column_name}" IS NULL',
        conn)['n'].iloc[0]
    conn.close()

    print()
    print(f"{'='*60}")
    action = "Would have updated" if dry_run else "Updated"
    print(f"  {action}: {updated} | Skipped: {skipped} | Errors: {errors}")
    print(f"  NULL remaining: {remaining}")
    if not dry_run and remaining == 0:
        print(f"  ✅ Column '{column_name}' fully populated!")
    print(f"{'='*60}")

    return 0 if remaining == 0 else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Targeted DB column repopulation (IMU-derived metrics only)')
    parser.add_argument(
        'column', nargs='?', default='imu_delta_v_z',
        help='Column name to populate (default: imu_delta_v_z)')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Preview only — no DB writes')
    args = parser.parse_args()

    sys.exit(repopulate_column(args.column, dry_run=args.dry_run))
