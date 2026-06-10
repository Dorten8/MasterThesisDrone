# Session Journal: 2026-06-09 — Battery Drain Rate Fix & Memory-Safe MCAP Streaming

## Where Are We in the Project

Pipeline database (`experiments_summary.db`) had garbage battery rates — per-pass MCAP windows are ~10 seconds, PX4 battery estimation is too coarse (1% steps), producing rates from -56% to +86%/min with 25% NULL. Flight-level rates needed from full unsliced MCAPs. Also, loading 200MB+ unsliced MCAPs into memory was triggering OOM → system freeze.

## Final State (21:15)

**Battery: 100% done.** `flights_summary` has **0 NULL battery columns**, 108 passes with realistic drain rates (21.3%/min both conditions).

## What Changed

### 1. Memory-Safe Streaming MCAP Reader (`db_unsliced_flights_bat_analyser.py`)

**Root cause of system freeze:** The old `load_mcap()` via `try_load_mcap()` collected ALL messages from ALL topics into Python objects. A 224MB MCAP has ~1.4M messages, each taking KBs as a Python object → 5+GB per MCAP. With multiple flights processed, this overwhelms even 32GB RAM.

**Fix:** New `_stream_load_for_battery()` function uses `read_ros2_messages(topics=[...])` to filter to only 4 needed topics during streaming:
- `/poses` — MoCap position (takeoff/landing detection via Z > 0.15m)
- `/fmu/out/battery_status` — voltage + remaining %
- `/fmu/out/vehicle_status` — arming/disarming state
- `/fmu/out/vehicle_odometry` — EKF velocity alignment

Messages are collected directly into lightweight `list[dict]` → converted to DataFrames at the end. Peak memory: ~350MB (vs 5+GB before). `gc.collect()` called between flights.

### 2. `flights_battery_efficiency` Table Populated (48/48 flights)

Ran `db_unsliced_flights_bat_analyser.py` with `force_recompute=True` on 48 unsliced MCAPs:
- **47 inserted directly**, 1 via ULog fallback (corrupted MCAP: `flight_20260529-1419`)
- **Fixed Cage:** 24.6%/min avg capacity drain, 1.06 V/min voltage drop, 104.8s avg flying
- **Rotating Cage:** 22.7%/min avg capacity drain, 0.60 V/min voltage drop, 123.9s avg flying
- These are physically realistic (user's longest flight: ~3.5 min, ~80% consumed ≈ 23%/min)
- **1 suspicious flight:** `flight_20260523-1919` — 0.0%/min (840s armed, 5s flying, battery jumps 48%→18%→41%). Data quality issue, zero impact — flight has no pass-sliced MCAPs, contributes 0 rows to `flights_summary`.

### 3. Pipeline Battery Lookup (`db_pipeline.py`)

Added `_get_battery_rates(flight_folder_name)` helper that queries `flights_battery_efficiency` table. Both pipeline paths (main `process_flights()` and secondary pass loop) now use this instead of computing per-pass from ~10s MCAP windows.

### 4. Pipeline Re-Run Complete

Modified `_rerun_pipeline.py` to only `DELETE FROM flights_summary` instead of deleting the entire DB file. Pipeline re-run completed:
- **108 passes total:** Fixed Cage 56, Rotating Cage 52
- **0 NULL battery columns** (6 NULL passes backfilled post-pipeline from ULog-fallback flight)
- **Avg drain:** 21.3%/min both conditions

### 5. ULog Fallback for Corrupted Flight

`flight_20260529-1419_45°_column_collision_loop_rotating_cage` — unsliced MCAP corrupted (opcode 28). Fixed via targeted ULog fallback:
- Parsed `14_13_41.ulg` for `vehicle_status` + `battery_status`
- Armed: 242.9s → Disarmed: 461.4s (218.4s armed)
- Battery: 24.5V / 100% → 22.7V / 40.6% → **16.6%/min** (consistent with Rotating Cage flights)
- 6 passes backfilled after pipeline finished → 0 NULLs remaining

### 6. Walkthrough Updated

`walkthrough_experiments.md` updated (ADD-only operations) with:
- CURRENT STATUS block at top with final verification commands
- Battery/Voltage Drop Fix section marked `[x]` with full explanation
- EKF Velocity Integration section marked `[x]`
- Representative Flight Finder marked `[x]` for implementation
- All NULL battery items marked `[x]`
- 2000 RPM outlier diagnosis with correction about motor_avg_before being actuator output (0-1), not RPM
- Pipeline Clean-Up Helper (SQL backfill command)

## Files Modified

| File | Change |
|------|--------|
| `dev_logs/analysis/database/db_unsliced_flights_bat_analyser.py` | Streaming reader + `gc.collect()`, removed old `try_load_mcap`/`build_dataframes` |
| `dev_logs/analysis/database/db_pipeline.py` | `_get_battery_rates()` lookup + both pipeline paths updated |
| `dev_logs/analysis/_rerun_pipeline.py` | Preserves `flights_battery_efficiency` on re-run |
| `dev_logs/analysis/walkthrough_experiments.md` | Battery/EKF/RepresentativeFinder status updates + verification commands (ADD-only) |
| `dev_logs/analysis/unsliced_battery_efficiency.csv` | Auto-exported from battery analyser (48 flights) |

## Files Created

| File | Purpose |
|------|---------|
| `dev_logs/analysis/_populate_battery_ulog_fallback.py` | ULog fallback for corrupted MCAP battery extraction |
| `dev_logs/session_journals/2026-06-09-battery-fix-and-pipeline-rerun.md` | This journal |

## Not Touched

- `dev_logs/analysis/experiments_analysis.ipynb` — explicitly preserved per user instruction

## Verification (Final State)

```bash
cd /home/dorten/MasterThesisDrone
python3 -c "
import sys; sys.path.insert(0, 'dev_logs/analysis/database')
sys.path.insert(0, '/home/dorten/.local/lib/python3.10/site-packages')
from db_manager import get_connection
c = get_connection().cursor()
c.execute('SELECT condition, COUNT(*), ROUND(AVG(capacity_drain_rate_flying),1), ROUND(AVG(total_flying_time),1) FROM flights_battery_efficiency GROUP BY condition')
for r in c.fetchall(): print(f'{r[0]}: {r[1]} flights, avg {r[2]}%/min, avg flying {r[3]}s')
c.execute('SELECT condition, COUNT(*), ROUND(AVG(capacity_drain_rate_pct_per_min),1) FROM flights_summary GROUP BY condition')
for r in c.fetchall(): print(f'{r[0]} (passes): {r[1]} rows, avg drain {r[2]}/min')
c.execute('SELECT COUNT(*) FROM flights_summary WHERE capacity_drain_rate_pct_per_min IS NULL')
print(f'NULL battery: {c.fetchone()[0]}')
"
```

Expected output:
```
Rotating Cage: X flights, avg ~22.7%/min, avg flying ~123s
Fixed Cage: Y flights, avg ~24.6%/min, avg flying ~104s
Rotating Cage (passes): 52 rows, avg drain 21.3/min
Fixed Cage (passes): 56 rows, avg drain 21.3/min
NULL battery: 0
```

---

## Late Addition (post-midnight, ~23:30): Star-Point Timing Investigation & SSoT Docs

### Where Are We

Battery and pipeline are done. Cutoff SSoT is established. But the "Exp. Start-point" vertical line on plots uses WP1 (command transition time), not when the drone actually starts moving — and at least one flight's trajectory plot looks physically impossible.

### 7. Approved Flight Cutoff SSoT — Documentation Cleanup

**Root cause:** The cutoff constant `APPROVED_CUTOFF = "20260524-1904"` was defined in two places (segmenter + pipeline) and not enforced in two others (battery analyser + re-run script).

**Fix:** Centralized in `db_manager.py` — added `APPROVED_CUTOFF`, `get_approved_cutoff()`, `is_approved_flight(folder_name)`. All four scripts now import and call `is_approved_flight()`. Pre-cutoff orphans (16 flights from May 23-24 with no pass MCAPs) deleted from `flights_battery_efficiency`.

**Files modified:**
- `dev_logs/analysis/database/db_manager.py` — SSoT added
- `dev_logs/analysis/database/db_mcap_event_segmenter.py` — import from SSoT
- `dev_logs/analysis/database/db_pipeline.py` — import from SSoT
- `dev_logs/analysis/database/db_unsliced_flights_bat_analyser.py` — cutoff gate added
- `dev_logs/analysis/_rerun_pipeline.py` — cutoff gate added

### 8. 75° Pipeline Re-Run — Missing Data Restored

**Root cause:** `_rerun_pipeline.py` ran `run("45deg", ...)` successfully (108 passes) then `run("75deg", ...)` which never completed. `DELETE FROM flights_summary` wiped old 75° data.

**Fix:** Ran targeted 75° pipeline re-run without the DELETE step → 71 passes restored. Database now has 179 passes total (108 × 45° + 71 × 75°).

### 9. Experiment Start-Point Timing — Investigation

**Problem:** The "Exp. Start-point" vertical line on plots is drawn at WP1's timestamp. WP1 is detected as the moment the flight director *transitions* to commanding WP2 — i.e., when the command to move is sent, not when the drone actually moves.

**Drone behavior issue:** The drone may drift off from the start point after it gets the command, then re-acquire position, and only then sprint forward. The start-point should therefore be when the drone *actually starts moving* (speed crossing ~0 → > 0.10 m/s).

**Current code state:**
- `find_waypoint_events()` in `kin_calculator.py` lines 560-587 already refines WP2 to `t_start_sweep` (last timestamp where MoCap speed < 0.10 m/s before crossing Y=0.70m). This is correct.
- BUT `draw_timeline_markers()` in `kin_plot_kinematics.py:14` uses `wp_events.get('WP1')` for the Exp. Start-point label — and **WP1 is NOT refined**, it stays as command transition time.

**Flight to inspect next session:** `flight_20260531-1112_45°_column_collision_loop_fixed_cage/pass02` — the IMU dynamics plot shows the timing, and the top-down trajectory plot looks "completely wrong and actually impossible."

**Solution (next session):** Build `dev_logs/scratch/plot_to_a4_pdf.py` — a script that collects all `pass*.png` plots → multi-page A4 PDF for physical printing + manual pass-by-pass validation.

**Files modified this session:**
| File | Change |
|------|--------|
| `.github/copilot-instructions.md` | Updated session status, added star-point timing section, updated priorities |
| `dev_logs/analysis/walkthrough_experiments.md` | Updated CURRENT STATUS block |
| `dev_logs/analysis/experiments_analysis_skill.md` | Added §3.0 cutoff documentation |

**Files created:**
| File | Purpose |
|------|---------|
| `/home/dorten/.claude/plans/add-here-the-plan-graceful-starlight.md` | Plan file for star-point timing fix + A4 PDF dumper |

**Not touched:**
- `dev_logs/analysis/experiments_analysis.ipynb` — explicitly preserved
- `experiments_summary.db` — cleaned (16 pre-cutoff rows removed, 75° restored)
