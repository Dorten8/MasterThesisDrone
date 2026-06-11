#!/usr/bin/env python3
"""One-shot: backup DB, wipe it, then rerun the full pipeline for all 45° and 75° flights.
   This forces recomputation of ALL metrics with EKF velocity + battery truncation.
"""
import os, sys, shutil, glob, re
from datetime import datetime

# ── Paths ──────────────────────────────────────────────────────────
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.expanduser("~/.local/lib/python3.10/site-packages"))

# Must match db_manager.py's DB_PATH exactly.
# db_manager.py: current_dir = .../analysis/database, .. = .../analysis
DB_PATH = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "experiments_summary.db"))
FLIGHTS_DIR = os.path.join(PROJECT_ROOT, "dev_logs", "flights")

# Import SSoT cutoff from db_manager
from dev_logs.analysis.database.db_manager import is_approved_flight

# ── 1. Discover flights (match _45°_ and _75°_ precisely) ──────────
rotating_45, fixed_45 = [], []
rotating_75, fixed_75 = [], []

for folder in sorted(os.listdir(FLIGHTS_DIR)):
    if not folder.startswith("flight_"):
        continue
    # ── Gate: approved date cutoff (SSoT from db_manager) ──
    if not is_approved_flight(folder):
        continue
    name_lower = folder.lower()
    if "rotating" in name_lower:
        if "_45°_" in folder:
            rotating_45.append(folder)
        elif "_75°_" in folder:
            rotating_75.append(folder)
    elif "fixed" in name_lower:
        if "_45°_" in folder:
            fixed_45.append(folder)
        elif "_75°_" in folder:
            fixed_75.append(folder)

print(f"Discovered flights:")
print(f"  45° Rotating Cage: {len(rotating_45)} flights  {rotating_45}")
print(f"  45° Fixed Cage:    {len(fixed_45)} flights  {fixed_45}")
print(f"  75° Rotating Cage: {len(rotating_75)} flights  {rotating_75}")
print(f"  75° Fixed Cage:    {len(fixed_75)} flights  {fixed_75}")
print(f"  Total: {len(rotating_45)+len(fixed_45)+len(rotating_75)+len(fixed_75)}")

# ── 2. Backup DB & wipe flights_summary (preserve flights_battery_efficiency) ──
import sqlite3
print(f"\n📁 DB path: {DB_PATH}")
if os.path.exists(DB_PATH):
    backup = DB_PATH + f".backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    shutil.copy2(DB_PATH, backup)
    print(f"📦 Backed up DB → {os.path.basename(backup)}")
    # Delete only flights_summary rows — keep flights_battery_efficiency intact
    db = sqlite3.connect(DB_PATH)
    db.execute("DELETE FROM flights_summary")
    db.commit()
    db.close()
    print(f"🗑️  Cleared flights_summary (preserved flights_battery_efficiency)")
else:
    print(f"ℹ️  No existing DB — fresh start")

# ── 3. Run pipeline ────────────────────────────────────────────────
from dev_logs.analysis.database.db_pipeline import run

for angle_label, angle_deg, cage_rot, cage_fix in [
    ("45deg", 45, rotating_45, fixed_45),
    ("75deg", 75, rotating_75, fixed_75),
]:
    print("\n" + "="*80)
    print(f"🚀 RUNNING: {angle_label} Pipeline")
    print(f"   Rotating: {len(cage_rot)} flights, Fixed: {len(cage_fix)} flights")
    print("="*80)

    run(angle_label, angle_deg,
        column_x=0.408, column_y=0.358,
        flights_rotating_cage=cage_rot,
        flights_fixed_cage=cage_fix,
        representative_rotating_cage=0,
        representative_fixed_cage=0,
        project_root=PROJECT_ROOT,
        force_plot=False)

print("\n" + "="*80)
print("✅ Pipeline re-run complete!")

# ── 4. Summary ─────────────────────────────────────────────────────
from dev_logs.analysis.database.db_manager import get_database_summary_markdown
print(get_database_summary_markdown())
