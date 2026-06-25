import sqlite3
import json
import os
import re
import datetime

# Resolve database path dynamically relative to the package location
current_dir = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.abspath(os.path.join(current_dir, "..", "experiments_summary.db"))

# ══════════════════════════════════════════════════════════════════════════════
# SSoT: Approved Flight Cutoff
# ══════════════════════════════════════════════════════════════════════════════
# Only flights with a folder timestamp >= this value (YYYYMMDD-HHMM) are
# eligible for analysis. Flights before this date were conducted before the
# MoCap tracking quality was stabilised (Fixed Cage flights suffered severe
# tracking dropouts at ~10 Hz, Rotating Cage at ~30 Hz), making their
# telemetry unreliable for comparative analysis.
#
# IMPORTED BY: db_pipeline.py, db_mcap_event_segmenter.py,
#              db_unsliced_flights_bat_analyser.py, rerun_pipeline.py
APPROVED_CUTOFF = "20260524-1904"


def get_approved_cutoff():
    """Return the SSoT approved flight cutoff string (YYYYMMDD-HHMM)."""
    return APPROVED_CUTOFF


def _extract_flight_timestamp(folder_name):
    """Extract YYYYMMDD-HHMM from a flight folder name, or None if unparseable."""
    # Flight folders follow the naming convention "flight_YYYYMMDD-HHMM_..."
    # The regex captures the date-time group \d{8}-\d{4} directly after "flight_".
    # This timestamp is used as a sortable string key for cutoff filtering,
    # since YYYYMMDD-HHMM compares lexicographically the same as chronologically.
    m = re.search(r'flight_(\d{8}-\d{4})', folder_name)
    return m.group(1) if m else None


def is_approved_flight(folder_name):
    """Return True if the flight folder timestamp is >= the approved cutoff."""
    # Lexicographic comparison works because the timestamp format
    # YYYYMMDD-HHMM is naturally sortable as a plain string.
    ts = _extract_flight_timestamp(folder_name)
    return ts is not None and ts >= APPROVED_CUTOFF


# ══════════════════════════════════════════════════════════════════════════════
# SSoT: Pass Exclusion Config
# ══════════════════════════════════════════════════════════════════════════════
# config_db.json records passes that manual IMU/trajectory review has deemed
# invalid: false positives (clearance negative but no real impact), borderline
# contacts (clearance -0.4 to 0.0 with barely visible impact), and data
# corruption (broken plots, missing path segments).
#
# The clearance-based impact_detected heuristic is a rough proxy — this config
# is the ground-truth override from human inspection.

CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_db.json")

_exclusion_cache = None


def _load_exclusion_config():
    """Load config_db.json, returning {db_pass_name: {excluded, reason, notes}}."""
    global _exclusion_cache
    # Cache once per process lifetime — the config is small and rarely changes.
    if _exclusion_cache is not None:
        return _exclusion_cache
    _exclusion_cache = {}
    if not os.path.exists(CONFIG_PATH):
        return _exclusion_cache
    with open(CONFIG_PATH, 'r') as f:
        cfg = json.load(f)
    # The JSON structure is: {"excluded_passes": {"folder_name": {"reason": "...", "passes": [1,3,5]}}}
    # For each numeric pass index we construct the canonical DB flight_name key
    # "folder_name - Pass-NN" so it exactly matches what insert_or_replace_flight() writes.
    excluded = cfg.get('excluded_passes', {})
    for folder_name, entry in excluded.items():
        if not isinstance(entry, dict):
            continue
        for pnum in entry.get('passes', []):
            db_name = f"{folder_name} - Pass-{pnum:02d}"
            _exclusion_cache[db_name] = {
                'excluded': 1,
                'exclusion_reason': entry.get('reason', 'unknown'),
                'notes': entry.get('notes', ''),
            }
    return _exclusion_cache


def is_excluded_pass(flight_name):
    """Return True if this pass is in the exclusion config (manual review override)."""
    cfg = _load_exclusion_config()
    return flight_name in cfg


def get_exclusion_info(flight_name):
    """Return (excluded, reason, notes) for a pass, or (0, None, None) if not excluded."""
    cfg = _load_exclusion_config()
    entry = cfg.get(flight_name)
    if entry:
        return entry['excluded'], entry['exclusion_reason'], entry.get('notes')
    return 0, None, None


def apply_exclusion_config_to_db():
    """Sync the exclusion config to the database: mark excluded passes.

    Reads config_db.json, UPDATEs flights_summary.excluded = 1 and sets
    exclusion_reason for every pass listed in excluded_passes.
    Does NOT remove rows — only marks them so queries can filter.
    """
    # This function is designed to be re-run safely after new passes are
    # inserted or after the JSON config is edited.  Only rows matching the
    # flight_name are touched; all other rows retain their current values.
    cfg = _load_exclusion_config()
    if not cfg:
        print("[EXCLUSION] No exclusion config found — nothing to sync.")
        return 0
    conn = get_connection()
    cursor = conn.cursor()
    updated = 0
    for db_name, info in cfg.items():
        cursor.execute(
            "UPDATE flights_summary SET excluded = ?, exclusion_reason = ? WHERE flight_name = ?",
            (info['excluded'], info['exclusion_reason'], db_name)
        )
        if cursor.rowcount > 0:
            updated += 1
    conn.commit()
    conn.close()
    print(f"[EXCLUSION] Marked {updated} passes as excluded in flights_summary.")
    return updated


def get_connection():
    """Establishes a resilient SQLite connection, preventing network mount locking conflicts."""
    conn = sqlite3.connect(DB_PATH, timeout=30.0)
    conn.execute("PRAGMA busy_timeout = 30000;")
    return conn

def init_db():
    """Initializes the SQLite database schema if it doesn't already exist.

    Design rationale:
    - A single flights_summary table stores per-pass metrics keyed by
      flight_name (which encodes folder + pass number).  This flat schema
      is deliberately denormalised — all metrics live in one wide row per
      pass so that downstream pandas/plotting code can SELECT * and get
      everything in one query without JOINs.

    Migration strategy (backwards-compatible):
    - The schema evolves over time as new IMU, motor, and allocator metrics
      are added.  Rather than requiring a destructive DROP + recreate, we
      use ALTER TABLE ADD COLUMN for each new field.  If the column already
      exists, sqlite3 raises OperationalError which we silently catch and
      ignore — this makes init_db() idempotent across schema versions.
    - Renamed columns (e.g. timestamp -> timestamp_db) are populated via
      UPDATE ... WHERE new_col IS NULL to copy old values without data loss.
    """
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = get_connection()
    cursor = conn.cursor()
    # ── Core flights_summary table (one row per pass) ──────────────────
    # flight_name is the PRIMARY KEY: "folder_name - Pass-NN" format.
    # condition encodes the experimental regime (e.g. "Fixed Cage").
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS flights_summary (
            flight_name      TEXT PRIMARY KEY,
            condition        TEXT NOT NULL,
            sweep_speed      REAL,
            battery_at_start REAL,
            impact_speed     REAL,
            before_impact_accel REAL,
            impact_accel     REAL,
            impact_angle     REAL,
            avg_dev_after    REAL,
            max_dev_after    REAL,
            recovery_area    REAL,
            closest_clearance REAL,
            impact_detected  INTEGER,
            nom_sp_x         REAL,
            nom_sp_y         REAL,
            nom_sp_z         REAL,
            nom_ep_x         REAL,
            nom_ep_y         REAL,
            nom_ep_z         REAL,
            act_sp_x         REAL,
            act_sp_y         REAL,
            act_sp_z         REAL,
            act_ep_x         REAL,
            act_ep_y         REAL,
            act_ep_z         REAL,
            imu_peak_accel   REAL,
            imu_peak_accel_x REAL,
            imu_peak_accel_y REAL,
            imu_peak_accel_z REAL,
            imu_peak_gyro    REAL,
            imu_peak_gyro_x  REAL,
            imu_peak_gyro_y  REAL,
            imu_peak_gyro_z  REAL,
            imu_accel_energy REAL,
            imu_accel_energy_x REAL,
            imu_accel_energy_y REAL,
            imu_accel_energy_z REAL,
            imu_gyro_energy  REAL,
            imu_gyro_energy_x REAL,
            imu_gyro_energy_y REAL,
            imu_gyro_energy_z REAL,
            imu_accel_settling REAL,
            imu_gyro_settling REAL,
            imu_std_ax       REAL,
            imu_std_ay       REAL,
            imu_std_az       REAL,
            imu_std_gx       REAL,
            imu_std_gy       REAL,
            imu_std_gz       REAL,
            timestamp_db                   TEXT,
            "e_sp_timestamp_PX4"            INTEGER,
            "e_impact_timestamp_PX4"        INTEGER,
            allocator_saturation_duration_sec REAL,
            max_unallocated_torque REAL,
            thrust_setpoint_achieved_pct REAL,
            roll_rate_error_rms REAL,
            pitch_rate_error_rms REAL,
            yaw_rate_error_rms REAL,
            active_flight_time_sec REAL,
            voltage_drop_rate_v_per_min REAL,
            capacity_drain_rate_pct_per_min REAL,
            max_actuator_output REAL,
            path_spread_rmsld REAL,
            imu_ax_spread_impact REAL,
            imu_ay_spread_impact REAL,
            imu_az_spread_impact REAL,
            imu_ax_spread_regular REAL,
            imu_ay_spread_regular REAL,
            imu_az_spread_regular REAL,
            excluded INTEGER DEFAULT 0,
            exclusion_reason TEXT
        )
    """)
    # ── Battery / efficiency auxiliary table ──────────────────────────
    # Stores per-FLIGHT (not per-pass) battery metrics — armed time,
    # voltage sag, capacity drain rates.  Separate table because these
    # metrics span the whole flight log, not individual column passes.
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS flights_battery_efficiency (
            flight_name                     TEXT PRIMARY KEY,
            condition                       TEXT NOT NULL,
            log_duration                    REAL,
            total_armed_time                REAL,
            total_flying_time               REAL,
            voltage_at_arm                  REAL,
            remaining_at_arm                REAL,
            voltage_at_takeoff              REAL,
            remaining_at_takeoff            REAL,
            voltage_at_landing              REAL,
            remaining_at_landing            REAL,
            voltage_at_disarm               REAL,
            remaining_at_disarm             REAL,
            min_voltage_during_flight       REAL,
            avg_voltage_during_flight       REAL,
            voltage_drop_rate_armed         REAL,
            capacity_drain_rate_armed       REAL,
            voltage_drop_rate_flying        REAL,
            capacity_drain_rate_flying      REAL,
            total_capacity_consumed_pct     REAL,
            total_voltage_dropped           REAL,
            timestamp_db                    TEXT
        )
    """)
    # ── Schema migration: RENAME old columns to new names ───────────────
    # 2026-06-25: Refactored imu_vib_* → imu_std_*, path_spread_sdld → path_spread_rmsld.
    # Uses PRAGMA table_info to detect old column names and ALTER TABLE RENAME COLUMN.
    # Idempotent: if old column doesn't exist, rename is skipped; if new column
    # already exists (fresh DB created with new schema), rename was already done.
    column_renames = [
        ("imu_vib_ax", "imu_std_ax"),
        ("imu_vib_ay", "imu_std_ay"),
        ("imu_vib_az", "imu_std_az"),
        ("imu_vib_gx", "imu_std_gx"),
        ("imu_vib_gy", "imu_std_gy"),
        ("imu_vib_gz", "imu_std_gz"),
        ("path_spread_sdld", "path_spread_rmsld"),
    ]
    existing_cols = {row[1] for row in cursor.execute("PRAGMA table_info(flights_summary)").fetchall()}
    for old_name, new_name in column_renames:
        if old_name in existing_cols and new_name not in existing_cols:
            try:
                cursor.execute(f'ALTER TABLE flights_summary RENAME COLUMN "{old_name}" TO "{new_name}"')
            except sqlite3.OperationalError:
                pass  # Already renamed in a previous run

    # ── Schema migration: add new columns idempotently ─────────────────
    # Each (column_name, type) pair is ALTER TABLE ADD COLUMN'd.
    # sqlite3.OperationalError is raised if the column already exists
    # (from a prior init_db run or newer DB file); we catch and skip
    # silently.  This means old databases opened by a newer version of the
    # code get all missing columns added with NULL defaults — no data loss.
    new_cols = [
        ("sweep_speed", "REAL"),
        ("battery_at_start", "REAL"),
        ("impact_detected", "INTEGER"),
        ("before_impact_accel", "REAL"),
        ("nom_sp_x", "REAL"),
        ("nom_sp_y", "REAL"),
        ("nom_sp_z", "REAL"),
        ("nom_ep_x", "REAL"),
        ("nom_ep_y", "REAL"),
        ("nom_ep_z", "REAL"),
        ("act_sp_x", "REAL"),
        ("act_sp_y", "REAL"),
        ("act_sp_z", "REAL"),
        ("act_ep_x", "REAL"),
        ("act_ep_y", "REAL"),
        ("act_ep_z", "REAL"),
        ("imu_peak_accel", "REAL"),
        ("imu_peak_accel_x", "REAL"),
        ("imu_peak_accel_y", "REAL"),
        ("imu_peak_accel_z", "REAL"),
        ("imu_peak_gyro", "REAL"),
        ("imu_peak_gyro_x", "REAL"),
        ("imu_peak_gyro_y", "REAL"),
        ("imu_peak_gyro_z", "REAL"),
        ("imu_accel_energy", "REAL"),
        ("imu_accel_energy_x", "REAL"),
        ("imu_accel_energy_y", "REAL"),
        ("imu_accel_energy_z", "REAL"),
        ("imu_gyro_energy", "REAL"),
        ("imu_gyro_energy_x", "REAL"),
        ("imu_gyro_energy_y", "REAL"),
        ("imu_gyro_energy_z", "REAL"),
        ("imu_accel_settling", "REAL"),
        ("imu_gyro_settling", "REAL"),
        ("imu_std_ax", "REAL"),
        ("imu_std_ay", "REAL"),
        ("imu_std_az", "REAL"),
        ("imu_std_gx", "REAL"),
        ("imu_std_gy", "REAL"),
        ("imu_std_gz", "REAL"),
        ("motor_avg_before", "REAL"),
        ("motor_max_before", "REAL"),
        ("motor_avg_after", "REAL"),
        ("motor_max_after", "REAL"),
        ("motor_thrust_surge", "REAL"),
        ("motor_imbalance_after", "REAL"),
        ("motor_m1_avg_after", "REAL"),
        ("motor_m2_avg_after", "REAL"),
        ("motor_m3_avg_after", "REAL"),
        ("motor_m4_avg_after", "REAL"),
        ("timestamp_db", "TEXT"),
        ("e_sp_timestamp_PX4", "INTEGER"),
        ("e_sp_timestamp_PX4_forw", "INTEGER"),
        ("e_ep_timestamp_PX4", "INTEGER"),
        ("e_impact_timestamp_PX4", "INTEGER"),
        ("allocator_saturation_duration_sec", "REAL"),
        ("max_unallocated_torque", "REAL"),
        ("thrust_setpoint_achieved_pct", "REAL"),
        ("yaw_rate_error_rms", "REAL"),
        ("active_flight_time_sec", "REAL"),
        ("voltage_drop_rate_v_per_min", "REAL"),
        ("capacity_drain_rate_pct_per_min", "REAL"),
        ("max_actuator_output", "REAL"),
        ("path_spread_rmsld", "REAL"),
        ("imu_ax_spread_impact", "REAL"),
        ("imu_ay_spread_impact", "REAL"),
        ("imu_az_spread_impact", "REAL"),
        ("imu_ax_spread_regular", "REAL"),
        ("imu_ay_spread_regular", "REAL"),
        ("imu_az_spread_regular", "REAL"),
        ("excluded", "INTEGER DEFAULT 0"),
        ("exclusion_reason", "TEXT")
    ]
    for col, coltype in new_cols:
        try:
            cursor.execute(f"ALTER TABLE flights_summary ADD COLUMN {col} {coltype}")
        except sqlite3.OperationalError:
            pass  # Column already exists
    # ── Data migration: copy old column values to renamed columns ──────
    # When a column was renamed (e.g. "timestamp" -> "timestamp_db"), the
    # ALTER TABLE ADD COLUMN above creates the new column with NULLs.
    # We then backfill: UPDATE ... SET new = old WHERE new IS NULL.
    # This is a one-way, non-destructive migration — old column is kept.
    for old, new in [("timestamp", "timestamp_db"),
                     ("timestamp_PX4", "e_sp_timestamp_PX4")]:
        try:
            cursor.execute(f'UPDATE flights_summary SET "{new}" = {old} WHERE "{new}" IS NULL')
        except sqlite3.OperationalError:
            pass
    # ── Battery table migration: timestamp → timestamp_db ─────────────
    try:
        cursor.execute("ALTER TABLE flights_battery_efficiency ADD COLUMN timestamp_db TEXT")
    except sqlite3.OperationalError:
        pass
    try:
        cursor.execute("UPDATE flights_battery_efficiency SET timestamp_db = timestamp WHERE timestamp_db IS NULL")
    except sqlite3.OperationalError:
        pass
    conn.commit()
    conn.close()

def is_already_cached(flight_name, check_columns=None):
    """Returns True if this pass is present in the database AND all specified check_columns are NOT NULL.

    Two-level cache check:
    1. Without check_columns: a simple SELECT 1 — the row exists, done.
    2. With check_columns: the row must exist AND every named column must be
       non-NULL.  This catches the case where a pass was inserted by an older
       version of the pipeline BEFORE certain metrics were added to the schema.
       The row exists (step 1 would return True) but the desired columns are
       still NULL because they were never computed for that pass.

    The sqlite3.OperationalError catch handles the edge case where a column
    name doesn't even exist in the schema yet (e.g. code references a brand-new
    metric before init_db() migration has been run on this DB file).
    """
    init_db()
    conn = get_connection()
    cursor = conn.cursor()
    if check_columns:
        cols_str = ", ".join(check_columns)
        try:
            cursor.execute(f"SELECT {cols_str} FROM flights_summary WHERE flight_name = ?", (flight_name,))
            row = cursor.fetchone()
            if row is None:
                is_cached = False
            else:
                # all() over the row values: every requested column must have
                # a real value (not NULL) for the pass to be considered cached
                is_cached = all(val is not None for val in row)
        except sqlite3.OperationalError:
            # Column doesn't exist yet (needs init/migration)
            is_cached = False
    else:
        cursor.execute("SELECT 1 FROM flights_summary WHERE flight_name = ?", (flight_name,))
        is_cached = cursor.fetchone() is not None
    conn.close()
    return is_cached

def insert_or_replace_flight(flight_name, condition, metrics):
    """Inserts or replaces a flight's metrics into the database."""
    init_db()

    # Extract values from metrics dictionary
    sweep_speed       = metrics.get('sweep_speed')
    battery_at_start  = metrics.get('battery_at_start')
    impact_speed      = metrics.get('impact_speed')
    before_impact_accel = metrics.get('before_impact_accel')
    impact_accel      = metrics.get('impact_accel')
    impact_angle      = metrics.get('achieved_impact_angle')
    avg_dev_after     = metrics.get('avg_dev_after')
    max_dev_after     = metrics.get('max_dev_after')
    recovery_area     = metrics.get('recovery_area')
    closest_clearance = metrics.get('closest_clearance')
    if closest_clearance is not None:
        closest_clearance_cm = closest_clearance * 100.0
    else:
        closest_clearance_cm = None

    # 18 new structural IMU metrics
    imu_peak_accel     = metrics.get('imu_peak_accel')
    imu_peak_accel_x   = metrics.get('imu_peak_accel_x')
    imu_peak_accel_y   = metrics.get('imu_peak_accel_y')
    imu_peak_accel_z   = metrics.get('imu_peak_accel_z')
    imu_peak_gyro      = metrics.get('imu_peak_gyro')
    imu_peak_gyro_x    = metrics.get('imu_peak_gyro_x')
    imu_peak_gyro_y    = metrics.get('imu_peak_gyro_y')
    imu_peak_gyro_z    = metrics.get('imu_peak_gyro_z')
    imu_accel_energy   = metrics.get('imu_accel_energy')
    imu_accel_energy_x = metrics.get('imu_accel_energy_x')
    imu_accel_energy_y = metrics.get('imu_accel_energy_y')
    imu_accel_energy_z = metrics.get('imu_accel_energy_z')
    imu_gyro_energy    = metrics.get('imu_gyro_energy')
    imu_gyro_energy_x  = metrics.get('imu_gyro_energy_x')
    imu_gyro_energy_y  = metrics.get('imu_gyro_energy_y')
    imu_gyro_energy_z  = metrics.get('imu_gyro_energy_z')
    imu_accel_settling = metrics.get('imu_accel_settling')
    imu_gyro_settling  = metrics.get('imu_gyro_settling')
    imu_std_ax         = metrics.get('imu_std_ax')
    imu_std_ay         = metrics.get('imu_std_ay')
    imu_std_az         = metrics.get('imu_std_az')
    imu_std_gx         = metrics.get('imu_std_gx')
    imu_std_gy         = metrics.get('imu_std_gy')
    imu_std_gz         = metrics.get('imu_std_gz')

    # Motor actuator metrics
    motor_avg_before   = metrics.get('motor_avg_before')
    motor_max_before   = metrics.get('motor_max_before')
    motor_avg_after    = metrics.get('motor_avg_after')
    motor_max_after    = metrics.get('motor_max_after')
    motor_thrust_surge = metrics.get('motor_thrust_surge')
    motor_imbalance_after = metrics.get('motor_imbalance_after')
    motor_m1_avg_after = metrics.get('motor_m1_avg_after')
    motor_m2_avg_after = metrics.get('motor_m2_avg_after')
    motor_m3_avg_after = metrics.get('motor_m3_avg_after')
    motor_m4_avg_after = metrics.get('motor_m4_avg_after')
    timestamp_PX4      = metrics.get('e_sp_timestamp_PX4')
    timestamp_PX4_forw = metrics.get('e_sp_timestamp_PX4_forw')
    e_ep_timestamp_PX4 = metrics.get('e_ep_timestamp_PX4')
    e_impact_timestamp_PX4 = metrics.get('e_impact_timestamp_PX4')

    allocator_saturation_duration_sec = metrics.get('allocator_saturation_duration_sec')
    max_unallocated_torque = metrics.get('max_unallocated_torque')
    thrust_setpoint_achieved_pct = metrics.get('thrust_setpoint_achieved_pct')
    roll_rate_error_rms = metrics.get('roll_rate_error_rms')
    pitch_rate_error_rms = metrics.get('pitch_rate_error_rms')
    yaw_rate_error_rms = metrics.get('yaw_rate_error_rms')
    
    active_flight_time_sec = metrics.get('active_flight_time_sec')
    voltage_drop_rate_v_per_min = metrics.get('voltage_drop_rate_v_per_min')
    capacity_drain_rate_pct_per_min = metrics.get('capacity_drain_rate_pct_per_min')
    max_actuator_output = metrics.get('max_actuator_output')
    path_spread_rmsld = metrics.get('path_spread_rmsld')
    
    imu_ax_spread_impact = metrics.get('imu_ax_spread_impact')
    imu_ay_spread_impact = metrics.get('imu_ay_spread_impact')
    imu_az_spread_impact = metrics.get('imu_az_spread_impact')
    imu_ax_spread_regular = metrics.get('imu_ax_spread_regular')
    imu_ay_spread_regular = metrics.get('imu_ay_spread_regular')
    imu_az_spread_regular = metrics.get('imu_az_spread_regular')

    # ── Impact detection heuristic ─────────────────────────────────────
    # This is a PROXY, not ground truth.  The definitive classification comes
    # from manual review stored in config_db.json (see exclusion config above).
    #
    # Heuristic: closest_clearance is the minimum Euclidean distance between
    # the drone MoCap position and the column center, minus the column radius.
    # A NEGATIVE clearance means the drone position penetrated the column's
    # geometric boundary — the simplest possible impact signal.
    #
    # Originally there was a secondary check for a deceleration spike
    # (≤ -1 m/s^2 within ±0.6 s of closest approach), but in practice the
    # negative-clearance test alone proved sufficient as a first-pass filter.
    # The manual review in config_db.json then overrides false positives.
    impact_detected = 0
    if closest_clearance_cm is not None and closest_clearance_cm < 0.0:
        impact_detected = 1  # Negative clearance = impact by default
        # Optionally confirm with deceleration spike (but negative clearance alone is sufficient)

    # Waypoints: Nominal vs Actual
    nom_sp = metrics.get('nom_sp', (None, None, None))
    nom_ep = metrics.get('nom_ep', (None, None, None))
    act_sp = metrics.get('act_sp', (None, None, None))
    act_ep = metrics.get('act_ep', (None, None, None))

    timestamp_db = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    conn = get_connection()
    cursor = conn.cursor()
    cols = [
        "flight_name", "condition", "sweep_speed", "battery_at_start",
        "impact_speed", "before_impact_accel", "impact_accel", "impact_angle",
        "avg_dev_after", "max_dev_after", "recovery_area", "closest_clearance",
        "impact_detected", 
        "nom_sp_x", "nom_sp_y", "nom_sp_z", 
        "nom_ep_x", "nom_ep_y", "nom_ep_z", 
        "act_sp_x", "act_sp_y", "act_sp_z", 
        "act_ep_x", "act_ep_y", "act_ep_z",
        "imu_peak_accel", "imu_peak_accel_x", "imu_peak_accel_y", "imu_peak_accel_z",
        "imu_peak_gyro", "imu_peak_gyro_x", "imu_peak_gyro_y", "imu_peak_gyro_z",
        "imu_accel_energy", "imu_accel_energy_x", "imu_accel_energy_y", "imu_accel_energy_z",
        "imu_gyro_energy", "imu_gyro_energy_x", "imu_gyro_energy_y", "imu_gyro_energy_z",
        "imu_accel_settling", "imu_gyro_settling",
        "imu_std_ax", "imu_std_ay", "imu_std_az",
        "imu_std_gx", "imu_std_gy", "imu_std_gz",
        "motor_avg_before", "motor_max_before", "motor_avg_after", "motor_max_after",
        "motor_thrust_surge", "motor_imbalance_after",
        "motor_m1_avg_after", "motor_m2_avg_after", "motor_m3_avg_after", "motor_m4_avg_after",
        "timestamp_db", "e_sp_timestamp_PX4", "e_sp_timestamp_PX4_forw", "e_ep_timestamp_PX4", "e_impact_timestamp_PX4",
        "allocator_saturation_duration_sec", "max_unallocated_torque", "thrust_setpoint_achieved_pct",
        "roll_rate_error_rms", "pitch_rate_error_rms", "yaw_rate_error_rms",
        "active_flight_time_sec", "voltage_drop_rate_v_per_min", "capacity_drain_rate_pct_per_min",
        "max_actuator_output", "path_spread_rmsld",
        "imu_ax_spread_impact", "imu_ay_spread_impact", "imu_az_spread_impact",
        "imu_ax_spread_regular", "imu_ay_spread_regular", "imu_az_spread_regular"
    ]

    vals = (
        flight_name, condition, sweep_speed, battery_at_start,
        impact_speed, before_impact_accel, impact_accel, impact_angle,
        avg_dev_after, max_dev_after, recovery_area, closest_clearance_cm,
        impact_detected, 
        nom_sp[0], nom_sp[1], nom_sp[2], 
        nom_ep[0], nom_ep[1], nom_ep[2], 
        act_sp[0], act_sp[1], act_sp[2], 
        act_ep[0], act_ep[1], act_ep[2],
        imu_peak_accel, imu_peak_accel_x, imu_peak_accel_y, imu_peak_accel_z,
        imu_peak_gyro, imu_peak_gyro_x, imu_peak_gyro_y, imu_peak_gyro_z,
        imu_accel_energy, imu_accel_energy_x, imu_accel_energy_y, imu_accel_energy_z,
        imu_gyro_energy, imu_gyro_energy_x, imu_gyro_energy_y, imu_gyro_energy_z,
        imu_accel_settling, imu_gyro_settling,
        imu_std_ax, imu_std_ay, imu_std_az,
        imu_std_gx, imu_std_gy, imu_std_gz,
        motor_avg_before, motor_max_before, motor_avg_after, motor_max_after,
        motor_thrust_surge, motor_imbalance_after,
        motor_m1_avg_after, motor_m2_avg_after, motor_m3_avg_after, motor_m4_avg_after,
        timestamp_db, timestamp_PX4, timestamp_PX4_forw, e_ep_timestamp_PX4, e_impact_timestamp_PX4,
        allocator_saturation_duration_sec, max_unallocated_torque, thrust_setpoint_achieved_pct,
        roll_rate_error_rms, pitch_rate_error_rms, yaw_rate_error_rms,
        active_flight_time_sec, voltage_drop_rate_v_per_min, capacity_drain_rate_pct_per_min,
        max_actuator_output, path_spread_rmsld,
        imu_ax_spread_impact, imu_ay_spread_impact, imu_az_spread_impact,
        imu_ax_spread_regular, imu_ay_spread_regular, imu_az_spread_regular
    )

    # ── INSERT OR REPLACE pattern ──────────────────────────────────────
    # Because flight_name is the PRIMARY KEY, INSERT OR REPLACE atomically
    # deletes the old row (if any) and inserts the new one in a single
    # statement.  This is the natural upsert for SQLite and means the
    # pipeline can be re-run on the same pass without manual cleanup —
    # re-computed metrics simply overwrite the previous values.
    query = f"INSERT OR REPLACE INTO flights_summary ({', '.join(cols)}) VALUES ({', '.join(['?'] * len(cols))})"
    cursor.execute(query, vals)
    conn.commit()
    conn.close()
    print(f"💾 Successfully cached '{flight_name}' metrics in SQLite database.")

def get_database_summary_markdown():
    """Queries all results from the SQLite database and returns them formatted as a Markdown table."""
    init_db()
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("""
        SELECT flight_name, condition, sweep_speed, battery_at_start,
               impact_speed, before_impact_accel, impact_accel, impact_angle,
               avg_dev_after, max_dev_after, recovery_area, closest_clearance,
               impact_detected, 
               nom_sp_x, nom_sp_y, nom_sp_z, 
               nom_ep_x, nom_ep_y, nom_ep_z, 
               act_sp_x, act_sp_y, act_sp_z, 
               act_ep_x, act_ep_y, act_ep_z
        FROM flights_summary
        ORDER BY condition DESC, flight_name ASC
    """)
    rows = cursor.fetchall()
    conn.close()

    if not rows:
        return "⚠️ SQLite database is empty."

    headers = [
        "Flight Name & Pass", "Condition", "Sweep Speed", "Battery Start",
        "Impact?", "Speed Impact (m/s)", "Before Accel (m/s²)", "Impact Accel (m/s²)", "Impact Angle",
        "Avg Dev (mm)", "Max Dev (mm)", "Recovery Area (cm²)", "Min Clearance (cm)",
        "Nominal SP (m)", "Actual SP (m)", "Nominal EP (m)", "Actual EP (m)"
    ]

    divider   = "| " + " | ".join(["---"] * len(headers)) + " |"
    md_header = "| " + " | ".join(headers) + " |"

    md_rows = []
    for r in rows:
        f_name    = r[0]
        cond      = r[1]
        sp        = f"{r[2]:.2f}" if r[2] is not None else "N/A"
        bat       = f"{r[3]:.1f}%" if r[3] is not None else "N/A"
        
        # Impact detection Yes/No
        impact_str = "Yes 💥" if r[12] == 1 else "No ⏭️"
        
        speed     = f"{r[4]:.3f}" if r[4] is not None else "N/A"
        b_acc     = f"{r[5]:.3f}" if r[5] is not None else "N/A"
        acc       = f"{r[6]:.3f}" if r[6] is not None else "N/A"
        angle     = f"{r[7]:.1f}°" if r[7] is not None else "N/A"
        avg_dev   = f"{r[8]:.1f}" if r[8] is not None else "N/A"
        max_dev   = f"{r[9]:.1f}" if r[9] is not None else "N/A"
        area      = f"{r[10]:.1f}" if r[10] is not None else "N/A"
        clearance = f"{r[11]:+.1f}" if r[11] is not None else "N/A"
        
        # Nom SP/EP
        nom_sp_coords = f"({r[13]:.3f}, {r[14]:.3f}, {r[15]:.3f})" if r[13] is not None else "N/A"
        nom_ep_coords = f"({r[16]:.3f}, {r[17]:.3f}, {r[18]:.3f})" if r[16] is not None else "N/A"
        
        # Act SP/EP
        act_sp_coords = f"({r[19]:.3f}, {r[20]:.3f}, {r[21]:.3f})" if r[19] is not None else "N/A"
        act_ep_coords = f"({r[22]:.3f}, {r[23]:.3f}, {r[24]:.3f})" if r[22] is not None else "N/A"

        md_rows.append(
            f"| {f_name} | {cond} | {sp} | {bat} | {impact_str} | {speed} | {b_acc} | {acc} | {angle} | {avg_dev} | {max_dev} | {area} | {clearance} | {nom_sp_coords} | {act_sp_coords} | {nom_ep_coords} | {act_ep_coords} |"
        )

    return "\n".join([md_header, divider] + md_rows)

def get_database_df():
    """Queries all results from the SQLite database and returns them as a pandas DataFrame.

    This is the primary bridge between the SQLite cache layer and the
    analysis/plotting code.  pd.read_sql_query() hands the SQL execution
    off to SQLite and constructs a DataFrame directly from the result set —
    column names and types are inferred from the table schema, and NULLs
    become NaN automatically.  The conn.close() in a finally block
    guarantees the connection is released even if the query fails.
    """
    import pandas as pd
    init_db()
    conn = get_connection()
    try:
        df = pd.read_sql_query("SELECT * FROM flights_summary ORDER BY condition DESC, flight_name ASC", conn)
    finally:
        conn.close()
    return df

def insert_or_replace_battery_efficiency(flight_name, condition, metrics):
    """Inserts or replaces raw flight battery and efficiency metrics in SQLite."""
    init_db()
    timestamp_db = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute("""
        INSERT OR REPLACE INTO flights_battery_efficiency (
            flight_name, condition, log_duration, total_armed_time, total_flying_time,
            voltage_at_arm, remaining_at_arm,
            voltage_at_takeoff, remaining_at_takeoff,
            voltage_at_landing, remaining_at_landing,
            voltage_at_disarm, remaining_at_disarm,
            min_voltage_during_flight, avg_voltage_during_flight,
            voltage_drop_rate_armed, capacity_drain_rate_armed,
            voltage_drop_rate_flying, capacity_drain_rate_flying,
            total_capacity_consumed_pct, total_voltage_dropped,
            timestamp_db
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, (
        flight_name, condition,
        metrics.get('log_duration'), metrics.get('total_armed_time'), metrics.get('total_flying_time'),
        metrics.get('voltage_at_arm'), metrics.get('remaining_at_arm'),
        metrics.get('voltage_at_takeoff'), metrics.get('remaining_at_takeoff'),
        metrics.get('voltage_at_landing'), metrics.get('remaining_at_landing'),
        metrics.get('voltage_at_disarm'), metrics.get('remaining_at_disarm'),
        metrics.get('min_voltage_during_flight'), metrics.get('avg_voltage_during_flight'),
        metrics.get('voltage_drop_rate_armed'), metrics.get('capacity_drain_rate_armed'),
        metrics.get('voltage_drop_rate_flying'), metrics.get('capacity_drain_rate_flying'),
        metrics.get('total_capacity_consumed_pct'), metrics.get('total_voltage_dropped'),
        timestamp_db
    ))
    conn.commit()
    conn.close()
    print(f"💾 Successfully cached '{flight_name}' battery efficiency metrics in SQLite.")

def get_battery_efficiency_df():
    """Queries all results from flights_battery_efficiency and returns them as a pandas DataFrame."""
    import pandas as pd
    init_db()
    conn = get_connection()
    try:
        df = pd.read_sql_query("SELECT * FROM flights_battery_efficiency ORDER BY condition DESC, flight_name ASC", conn)
    finally:
        conn.close()
    return df

