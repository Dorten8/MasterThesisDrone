import sqlite3
import os
import datetime

# Resolve database path dynamically relative to the package location
current_dir = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.abspath(os.path.join(current_dir, "..", "experiments_summary.db"))

def get_connection():
    """Establishes a resilient SQLite connection, preventing network mount locking conflicts."""
    conn = sqlite3.connect(DB_PATH, timeout=30.0)
    conn.execute("PRAGMA busy_timeout = 30000;")
    return conn

def init_db():
    """Initializes the SQLite database schema if it doesn't already exist."""
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = get_connection()
    cursor = conn.cursor()
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
            imu_vib_ax       REAL,
            imu_vib_ay       REAL,
            imu_vib_az       REAL,
            imu_vib_gx       REAL,
            imu_vib_gy       REAL,
            imu_vib_gz       REAL,
            timestamp        TEXT,
            timestamp_PX4    INTEGER,
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
            path_spread_sdld REAL,
            imu_ax_spread_impact REAL,
            imu_ay_spread_impact REAL,
            imu_az_spread_impact REAL,
            imu_ax_spread_regular REAL,
            imu_ay_spread_regular REAL,
            imu_az_spread_regular REAL
        )
    """)
    # Initialize raw flight battery and efficiency table
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
            timestamp                       TEXT
        )
    """)
    # Migrate: add new columns if they don't yet exist (backwards compat with old DB)
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
        ("imu_vib_ax", "REAL"),
        ("imu_vib_ay", "REAL"),
        ("imu_vib_az", "REAL"),
        ("imu_vib_gx", "REAL"),
        ("imu_vib_gy", "REAL"),
        ("imu_vib_gz", "REAL"),
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
        ("timestamp_PX4", "INTEGER"),
        ("allocator_saturation_duration_sec", "REAL"),
        ("max_unallocated_torque", "REAL"),
        ("thrust_setpoint_achieved_pct", "REAL"),
        ("yaw_rate_error_rms", "REAL"),
        ("active_flight_time_sec", "REAL"),
        ("voltage_drop_rate_v_per_min", "REAL"),
        ("capacity_drain_rate_pct_per_min", "REAL"),
        ("max_actuator_output", "REAL"),
        ("path_spread_sdld", "REAL"),
        ("imu_ax_spread_impact", "REAL"),
        ("imu_ay_spread_impact", "REAL"),
        ("imu_az_spread_impact", "REAL"),
        ("imu_ax_spread_regular", "REAL"),
        ("imu_ay_spread_regular", "REAL"),
        ("imu_az_spread_regular", "REAL")
    ]
    for col, coltype in new_cols:
        try:
            cursor.execute(f"ALTER TABLE flights_summary ADD COLUMN {col} {coltype}")
        except sqlite3.OperationalError:
            pass  # Column already exists
    conn.commit()
    conn.close()

def is_already_cached(flight_name, check_columns=None):
    """Returns True if this pass is present in the database AND all specified check_columns are NOT NULL."""
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
    imu_vib_ax         = metrics.get('imu_vib_ax')
    imu_vib_ay         = metrics.get('imu_vib_ay')
    imu_vib_az         = metrics.get('imu_vib_az')
    imu_vib_gx         = metrics.get('imu_vib_gx')
    imu_vib_gy         = metrics.get('imu_vib_gy')
    imu_vib_gz         = metrics.get('imu_vib_gz')

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
    timestamp_PX4      = metrics.get('timestamp_PX4')

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
    path_spread_sdld = metrics.get('path_spread_sdld')
    
    imu_ax_spread_impact = metrics.get('imu_ax_spread_impact')
    imu_ay_spread_impact = metrics.get('imu_ay_spread_impact')
    imu_az_spread_impact = metrics.get('imu_az_spread_impact')
    imu_ax_spread_regular = metrics.get('imu_ax_spread_regular')
    imu_ay_spread_regular = metrics.get('imu_ay_spread_regular')
    imu_az_spread_regular = metrics.get('imu_az_spread_regular')

    # Impact Heuristic (simplified):
    # 1. Primary: closest_clearance must be negative (cage penetrated column boundary)
    # 2. Secondary: look for a deceleration spike of magnitude ≤ -1 m/s² within ±0.6s of closest approach
    impact_detected = 0
    if closest_clearance_cm is not None and closest_clearance_cm < 0.0:
        impact_detected = 1  # Negative clearance = impact by default
        # Optionally confirm with deceleration spike (but negative clearance alone is sufficient)

    # Waypoints: Nominal vs Actual
    nom_sp = metrics.get('nom_sp', (None, None, None))
    nom_ep = metrics.get('nom_ep', (None, None, None))
    act_sp = metrics.get('act_sp', (None, None, None))
    act_ep = metrics.get('act_ep', (None, None, None))

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

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
        "imu_vib_ax", "imu_vib_ay", "imu_vib_az",
        "imu_vib_gx", "imu_vib_gy", "imu_vib_gz",
        "motor_avg_before", "motor_max_before", "motor_avg_after", "motor_max_after",
        "motor_thrust_surge", "motor_imbalance_after",
        "motor_m1_avg_after", "motor_m2_avg_after", "motor_m3_avg_after", "motor_m4_avg_after",
        "timestamp", "timestamp_PX4",
        "allocator_saturation_duration_sec", "max_unallocated_torque", "thrust_setpoint_achieved_pct",
        "roll_rate_error_rms", "pitch_rate_error_rms", "yaw_rate_error_rms",
        "active_flight_time_sec", "voltage_drop_rate_v_per_min", "capacity_drain_rate_pct_per_min",
        "max_actuator_output", "path_spread_sdld",
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
        imu_vib_ax, imu_vib_ay, imu_vib_az,
        imu_vib_gx, imu_vib_gy, imu_vib_gz,
        motor_avg_before, motor_max_before, motor_avg_after, motor_max_after,
        motor_thrust_surge, motor_imbalance_after,
        motor_m1_avg_after, motor_m2_avg_after, motor_m3_avg_after, motor_m4_avg_after,
        timestamp, timestamp_PX4,
        allocator_saturation_duration_sec, max_unallocated_torque, thrust_setpoint_achieved_pct,
        roll_rate_error_rms, pitch_rate_error_rms, yaw_rate_error_rms,
        active_flight_time_sec, voltage_drop_rate_v_per_min, capacity_drain_rate_pct_per_min,
        max_actuator_output, path_spread_sdld,
        imu_ax_spread_impact, imu_ay_spread_impact, imu_az_spread_impact,
        imu_ax_spread_regular, imu_ay_spread_regular, imu_az_spread_regular
    )

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
    """Queries all results from the SQLite database and returns them as a pandas DataFrame."""
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
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
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
            timestamp
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
        timestamp
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

