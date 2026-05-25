import sqlite3
import os
import datetime

# Resolve database path dynamically relative to the package location
current_dir = os.path.dirname(os.path.abspath(__file__))
DB_PATH = os.path.abspath(os.path.join(current_dir, "..", "collision_experiments.db"))

def init_db():
    """Initializes the SQLite database schema if it doesn't already exist."""
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS flights_summary (
            flight_name      TEXT PRIMARY KEY,
            condition        TEXT NOT NULL,
            sweep_speed      REAL,
            battery_at_start REAL,
            impact_speed     REAL,
            impact_accel     REAL,
            impact_angle     REAL,
            avg_dev_after    REAL,
            max_dev_after    REAL,
            recovery_area    REAL,
            closest_clearance REAL,
            timestamp        TEXT
        )
    """)
    # Migrate: add new columns if they don't yet exist (backwards compat with old DB)
    for col, coltype in [("sweep_speed", "REAL"), ("battery_at_start", "REAL")]:
        try:
            cursor.execute(f"ALTER TABLE flights_summary ADD COLUMN {col} {coltype}")
        except sqlite3.OperationalError:
            pass  # Column already exists
    conn.commit()
    conn.close()

def is_already_cached(flight_name):
    """Returns True if this pass is already present in the database."""
    init_db()
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT 1 FROM flights_summary WHERE flight_name = ?", (flight_name,))
    exists = cursor.fetchone() is not None
    conn.close()
    return exists

def insert_or_replace_flight(flight_name, condition, metrics):
    """Inserts or replaces a flight's metrics into the database."""
    init_db()

    # Extract values from metrics dictionary
    sweep_speed       = metrics.get('sweep_speed')
    battery_at_start  = metrics.get('battery_at_start')
    impact_speed      = metrics.get('impact_speed')
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

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        INSERT OR REPLACE INTO flights_summary (
            flight_name, condition, sweep_speed, battery_at_start,
            impact_speed, impact_accel, impact_angle,
            avg_dev_after, max_dev_after, recovery_area, closest_clearance, timestamp
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, (
        flight_name, condition, sweep_speed, battery_at_start,
        impact_speed, impact_accel, impact_angle,
        avg_dev_after, max_dev_after, recovery_area, closest_clearance_cm, timestamp
    ))
    conn.commit()
    conn.close()
    print(f"💾 Successfully cached '{flight_name}' metrics in SQLite database.")

def get_database_summary_markdown():
    """Queries all results from the SQLite database and returns them formatted as a Markdown table."""
    init_db()
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("""
        SELECT flight_name, condition, sweep_speed, battery_at_start,
               impact_speed, impact_accel, impact_angle,
               avg_dev_after, max_dev_after, recovery_area, closest_clearance
        FROM flights_summary
        ORDER BY condition DESC, flight_name ASC
    """)
    rows = cursor.fetchall()
    conn.close()

    if not rows:
        return "⚠️ SQLite database is empty."

    headers = [
        "Flight Name & Pass", "Condition", "Sweep Speed (m/s)", "Battery at Start (%)",
        "Speed at Impact (m/s)", "Accel (m/s²)", "Impact Angle (°)",
        "Avg Dev After (mm)", "Max Dev After (mm)", "Recovery Area (cm²)", "Min Clearance (cm)"
    ]

    divider   = "| " + " | ".join(["---"] * len(headers)) + " |"
    md_header = "| " + " | ".join(headers) + " |"

    md_rows = []
    for r in rows:
        f_name    = r[0]
        cond      = r[1]
        sp        = f"{r[2]:.2f}" if r[2] is not None else "N/A"
        bat       = f"{r[3]:.1f}%" if r[3] is not None else "N/A"
        speed     = f"{r[4]:.3f}" if r[4] is not None else "N/A"
        acc       = f"{r[5]:.3f}" if r[5] is not None else "N/A"
        angle     = f"{r[6]:.1f}°" if r[6] is not None else "N/A"
        avg_dev   = f"{r[7]:.1f}" if r[7] is not None else "N/A"
        max_dev   = f"{r[8]:.1f}" if r[8] is not None else "N/A"
        area      = f"{r[9]:.1f}" if r[9] is not None else "N/A"
        clearance = f"{r[10]:+.1f}" if r[10] is not None else "N/A"

        md_rows.append(
            f"| {f_name} | {cond} | {sp} | {bat} | {speed} | {acc} | {angle} | {avg_dev} | {max_dev} | {area} | {clearance} |"
        )

    return "\n".join([md_header, divider] + md_rows)
