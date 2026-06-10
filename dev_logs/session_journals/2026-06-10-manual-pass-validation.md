# Session Journal: 2026-06-10 — Manual Pass Validation & Exclusion Pipeline

## Where Are We in the Project

Finished the manual pass-by-pass IMU/trajectory plot review using the A3 print dump (179 passes across 3 volumes). Discovered that the clearance-based `impact_detected` heuristic has a ~6% false-positive rate at the boundary (-0.4 to 0.0 cm). Built a configuration-driven exclusion system to keep the database and analysis clean.

## What We Reviewed

Printed all 179 passes from the A3 PDF and manually inspected each pass's IMU dynamics plot to verify whether a real collision occurred, comparing against the DB's `impact_detected` flag (based on `closest_clearance < 0.0`).

### Clearance Heuristic Validation

From the user's manual review, the clearance-to-impact mapping:

| Clearance | Verdict | Reliability |
|-----------|---------|-------------|
| ≤ -0.5 cm | Definitely impacted | High — no false negatives observed |
| -0.4 to 0.0 cm | **Unclear / borderline** | Some passes impacted, some didn't — needs IMU confirmation |
| ≥ 0.0 cm | Definitely NOT impacted | High — but one case (clearance 0.0) still had slight contact |

### 11 Passes Identified for Exclusion

**No Impact (false positives — clearance negative but IMU shows no collision):**
1. `flight_20260525-1716_75°_fixed_cage` Pass 07 — clearance +0.2, visual angle ~90° (not 75°)
2. `flight_20260526-0922_75°_rotating_cage` Pass 01 — clearance -0.3, no impact (contrast: Pass 04 in same flight, -0.4, DID impact)
3. `flight_20260526-0931_75°_rotating_cage` Pass 02 — clearance -0.1, no impact
4. `flight_20260526-0931_75°_rotating_cage` Pass 04 — clearance 0.0, very marginal
5. `flight_20260528-1230_75°_fixed_cage` Pass 01 — clearance -0.4, very slight impact
6. `flight_20260528-1230_75°_fixed_cage` Pass 04 — clearance -0.3, almost no impact
7. `flight_20260528-1230_75°_fixed_cage` Pass 06 — clearance -0.3, no impact

**Slight Impact (borderline — visible but very weak):**
8. `flight_20260528-1239_75°_rotating_cage` Pass 02 — clearance -0.3
9. `flight_20260528-1239_75°_rotating_cage` Pass 03 — clearance -0.4

**Slight Impact + Data Quality Issues:**
10. `flight_20260529-1419_45°_rotating_cage` Pass 05 — clearance -0.2, slight impact (same flight had corrupted unsliced MCAP)

**Data Corruption (broken plot):**
11. `flight_20260531-1228_45°_rotating_cage` Pass 05 — path after impact missing, plot is broken

### Interesting Counter-Examples (Clear Impact Despite Marginal Clearance — KEPT)

These demonstrate that clearance alone isn't destiny — the cage can still transfer significant force even at "borderline" clearance values:

- `flight_20260526-0922_75°_rotating_cage` Pass 04 — clearance -0.4, **definitely impacted** (vs Pass 01: -0.3, no impact — 1 mm difference)
- `flight_20260528-1655_45°_fixed_cage` Pass 01/02 — clearance -0.3, **very clear impact**
- `flight_20260529-1049_45°_fixed_cage` Pass 05 — clearance -0.1, **super clear impact**

These are flagged in `config_db.json` under `flagged_passes` for awareness but kept in the dataset.

## What Changed

### 1. Exclusion Config — SSoT (`dev_logs/analysis/database/config_db.json`)

Created a structured JSON configuration with two sections:
- **`excluded_passes`**: 11 passes across 7 flights — hierarchically organized by flight folder, with pass numbers, exclusion reason, and detailed notes
- **`flagged_passes`**: 4 counter-example passes — marginal clearance but clear impact, kept in dataset but noted

Three exclusion categories defined:
- `no_impact_false_positive` — DB says impact=1 but IMU shows no real collision
- `slight_impact_borderline` — impact barely visible, excluded for clean analysis
- `broken_page_data_corruption` — plot/data is corrupted, unusable

Rules of thumb documented in the config:
- Clearance ≤ -0.5 cm → definitely impacted
- Clearance -0.4 to 0.0 cm → unclear, needs IMU review
- Clearance ≥ 0.0 cm → definitely not impacted

### 2. Database Schema — `excluded` + `exclusion_reason` Columns

Added to `flights_summary` in `db_manager.py`:
```sql
excluded INTEGER DEFAULT 0,
exclusion_reason TEXT
```

New helper functions in `db_manager.py`:
- `_load_exclusion_config()` — parses `config_db.json`, builds `{db_pass_name: info}` dict
- `is_excluded_pass(flight_name)` → bool
- `get_exclusion_info(flight_name)` → (excluded, reason, notes)
- `apply_exclusion_config_to_db()` — syncs config to DB: `UPDATE flights_summary SET excluded=1, exclusion_reason=...` for all listed passes

Config is cached in memory after first load — no repeated file I/O.

### 3. Pipeline Gate (`db_pipeline.py`)

Both code paths (the `process_flights()` function and the standalone `if __name__ == "__main__"` runner) now check `is_excluded_pass()` before processing. Excluded passes are skipped with a log message:
```
⏭️  'flight_... - Pass-NN' excluded by manual review (no_impact_false_positive), skipping.
```

Import chain: `db_pipeline.py` now imports `is_excluded_pass` and `apply_exclusion_config_to_db` from `db_manager.py`.

### 4. A3 PDF Script — Impacted-Only Filter (`plot_to_a3_pdf.py`)

Three changes:
1. **New `_load_db_impact_filter()`** — queries `flights_summary` for passes where `impact_detected = 1 AND (excluded IS NULL OR excluded = 0)`, returns a set of `(folder_name, pass_num)` tuples
2. **`discover_passes()` now takes `impact_only=True`** — cross-references the DB filter against PNG-discovered passes. With `impact_only=True` (default), non-impacted and manually excluded passes are silently dropped
3. **New `--include-all` flag** — bypasses the DB filter, dumps everything with PNGs (the old behavior)

### 5. A3 PDF Rebuilt

Old output: 179 passes, 3 volumes, 97.6 MB total (150 DPI)
New output: **127 passes, 2 volumes, 69 MB total** (150 DPI)
- `all_passes_a3_v1of2.pdf` — 84 passes, 85 pages, 45.6 MB
- `all_passes_a3_v2of2.pdf` — 43 passes, 44 pages, 23.4 MB

Breakdown: 67 Fixed Cage + 60 Rotating Cage, all verified-impact + non-excluded.

## Final DB State

```
Total passes:           179
  impact_detected = 1:  137 (clearance < 0)
  impact_detected = 0:   42 (no cage penetration)
  manually excluded:     11 (from manual IMU review)
  clean impacted:       127 (analysis-ready)
```

## Files Modified

| File | Change |
|------|--------|
| `dev_logs/analysis/database/config_db.json` | **Created** — SSoT exclusion config with 11 excluded + 4 flagged passes |
| `dev_logs/analysis/database/db_manager.py` | `import json`; added `excluded`/`exclusion_reason` columns to schema + migration; added `CONFIG_PATH`, `_exclusion_cache`, `_load_exclusion_config()`, `is_excluded_pass()`, `get_exclusion_info()`, `apply_exclusion_config_to_db()` |
| `dev_logs/analysis/database/db_pipeline.py` | Import `is_excluded_pass`, `apply_exclusion_config_to_db`; exclusion gate in `process_flights()` and standalone runner |
| `dev_logs/scratch/plot_to_a3_pdf.py` | Added `DB_PATH`, `_load_db_impact_filter()`, `impact_only` param to `discover_passes()`, `--include-all` flag |

## Files Created

| File | Purpose |
|------|---------|
| `dev_logs/analysis/database/config_db.json` | Ground-truth exclusion/flagging config from manual pass review |
| `dev_logs/session_journals/2026-06-10-manual-pass-validation.md` | This journal |

## Not Touched

- `dev_logs/analysis/experiments_analysis.ipynb` — explicitly preserved
- `experiments_summary.db` — modified in-place (11 rows updated, no deletions)

## Verification

```bash
cd /home/dorten/MasterThesisDrone
python3 -c "
import sys; sys.path.insert(0, 'dev_logs/analysis/database')
sys.path.insert(0, '/home/dorten/.local/lib/python3.10/site-packages')
from db_manager import get_connection
c = get_connection().cursor()
c.execute('SELECT excluded, exclusion_reason, COUNT(*) FROM flights_summary WHERE excluded=1 GROUP BY excluded, exclusion_reason')
for r in c.fetchall(): print(f'excluded=1, reason={r[1]}: {r[2]} passes')
c.execute('SELECT COUNT(*) FROM flights_summary WHERE impact_detected=1 AND (excluded IS NULL OR excluded=0)')
print(f'Clean impacted passes: {c.fetchone()[0]}')
"
```

Expected output:
```
excluded=1, reason=no_impact_false_positive: 7 passes
excluded=1, reason=slight_impact_borderline: 3 passes
excluded=1, reason=broken_page_data_corruption: 1 pass
Clean impacted passes: 127
```

## Learning Summary

1. **The clearance heuristic is ~92% accurate.** Of 137 passes with negative clearance, 11 (~8%) were false positives — no real impact despite cage penetrating the column boundary. The failure mode is concentrated in the -0.4 to 0.0 cm range.

2. **1 mm can make the difference between hit and miss.** `flight_20260526-0922` rotating cage: Pass 01 at -0.3 cm missed entirely, Pass 04 at -0.4 cm had a definite impact. The cage radius is 17.9 cm — this is a 0.05% difference relative to cage diameter.

3. **Clearance direction can invert the relationship.** `flight_20260529-1049` Pass 05 had clearance of only -0.1 cm but a "super clear" impact, while other passes with deeper clearance showed less IMU response. Cage orientation, spin, and exact contact point matter as much as penetration depth.

4. **Manual plot review catches what heuristics miss.** Building the A3 print dump and physically reviewing each pass was essential — no automated metric could distinguish "cage touched column but drone didn't react" from "real collision."

## Next Steps

1. The 127 clean passes are ready for thesis analysis — all statistical comparisons should use `WHERE excluded = 0 AND impact_detected = 1`
2. Consider whether the 3 `slight_impact_borderline` passes should be re-evaluated (they're close calls — could argue either way)
3. The clearance heuristic could be improved: require `closest_clearance ≤ -0.5` OR `imu_peak_accel > threshold` for impact detection
4. The counter-examples (clear impact at marginal clearance) are interesting for the Discussion section — demonstrates that clearance alone doesn't predict collision severity

---

*Committed 2026-06-10 — `config_db.json`, db_manager.py/db_pipeline.py exclusion gates, session journal, and DB migration.*
