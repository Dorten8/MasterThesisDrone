# Session Journal: 2026-05-23 — MoCap Recovery & Experimental Analysis Framework Design (20-30% Thesis Pipeline)

## Where Are We in the Project

Continuing autonomous drone control with MoCap-based localization and thesis-grade collision experiments. Today's session marks a major milestone:
1. **Experimental Analysis Framework (20-30% Thesis Pipeline)**: Designed and implemented the core scientific data pipeline that automatically ingests ROS 2 MCAP bags, cleans high-frequency MoCap positions using Savitzky-Golay filtering, segments flights chronologically by waypoint, calculates precise physical metrics (closest approach clearance, tracking error, velocity profiles), and compiles comparative statistical graphics (With Cage vs. Without Cage) across flight sweeps.
2. **MoCap Recovery & Diagnostics**: Debugged and resolved startup failures after the OptiTrack PC rebooted due to a power outage, successfully eliminating hardcoded network parameters and introducing robust companion-to-FC health diagnostics.
3. **75° Collision Loop Optimization**: Refactored the `exp_collision_75deg.py` mission, renaming the class to `ExpCollision75Deg`, enforcing explicit `1.0 m/s` transit/recovery safety speeds, and symmetrizing the flight path to a strictly vertical straight line parallel to the Y-axis at $X = -0.186$ m, matching your CAD sketch and `/poses` column offset.

---

## What We Worked On and Why

### 1. Development of the Experimental Analysis Framework (Thesis Core Milestone)
**What:** Conceived, designed, and implemented the core mathematical and statistical pipeline for the drone's collision experiments. This framework digests raw ROS 2 MCAP bags, reconstructs the flight kinematics, automatically detects waypoints and active sweep segments, and compiles multi-pass comparative statistics (With Cage vs. Without Cage) and multi-angle progression trends.

**Why:** This represents the scientific "meat, bone, and fat" of the Master's Thesis. Prior to this, there was no way to systematically parse, clean, filter, segment, and compare experimental runs. Without an automated pipeline, analyzing 5 passes with the cage and 5 passes without the cage across 7 different sweep angles (70 total flights!) would require weeks of manual data cleaning and plotting, creating a massive research bottleneck. This framework advances the thesis analytical dashboard to roughly 20-30% of its final complete version.

**How it compares:** Previously, we had basic, loose script-plotting cells in a single Jupyter notebook that only parsed one flight at a time. The new pipeline is a fully realized, parameter-driven python library (`experiments_analysis`) that orchestrates MoCap signal processing, spatial geometry error tracking, event segmentation, and publication-ready comparative statistics in a unified pipeline. 

**Hurdles & Details:** 
* **Noise and Derivatives:** Raw numerical derivatives on high-frequency MoCap positions yielded extremely noisy velocity signals. *Solution:* Integrated a Savitzky-Golay filter (`scipy.signal.savgol_filter`) to compute smooth, physical velocities.
* **Temporal Segmentation:** Flights are continuous, but analysis must only look at the active sweep segment (`WP2 -> WP3`). *Solution:* Implemented threshold-based event detection to automatically segment and isolate each pass, logging precise waypoint entry/exit journals.
* **Tracking Error Calculations:** Needed a way to quantify flight stability during the sweep. *Solution:* Designed perpendicular tracking error calculation (perpendicular distance of the drone center to the desired straight-line path).
* **Multi-Pass & Multi-Angle Comparisons:** Standard box plots weren't sufficient to show cross-flight variance and trend progression. *Solution:* Built a 2x2 statistical comparative grid comparing "With Cage" vs "Without Cage" side-by-side (using both box plots and velocity envelopes), and engineered line-chart progression trends to map performance as the sweep angle changes.

### 2. Hardcoded IP Addresses in Startup Script
**What:** The `startup-sequence.sh` was mixing hardcoded IP addresses (`192.168.74.2`) with config-driven values for the OptiTrack server IP.

**Why:** When the OptiTrack PC's IP changes (due to DHCP, reboot, or network changes), the hardcoded fallback wouldn't match the actual system, causing silent failures.

**How it compares:** Previously, these addresses were scattered with no single source of truth; now all network configuration reads dynamically from `config/drone_config.json`.

**Hurdles & Solutions:** Updated the startup script to read `optitrack_server_ip` from config for both MoCap server connection and NTP fallback IP, removing all hardcoded IPs.

**Solution:** Updated the startup script to read `optitrack_server_ip` from config for both MoCap server connection and NTP fallback IP, removing all hardcoded IPs.

### 2. Startup Diagnostics for FC Stale Connection
**What:** Added a health-check diagnostic after `mocap_px4_bridge` startup that distinguishes between MicroXRCEAgent connection failures and MoCap system issues.

**Why:** When debugging, it was previously unclear whether failures were due to stale FC connections (requiring hardware reboot) or MoCap system issues. The diagnostic directly checks for `/fmu/out/*` topics (proves FC connection) and `/poses` topic existence (proves MoCap is running).

**How it compares:** Previously, silent timeouts gave no actionable guidance; now the summary explicitly states "Power cycle the drone" or "Motion capture system issue."

**Hurdle:** Initial diagnostic was checking for pose data in the `/poses` topic, but that was misleading—the topic could exist without data. Changed to check for `/fmu/out/*` topics instead.

**Solution:** Updated the diagnostic to run:
```bash
FMU_TOPICS=$(ros2 topic list | grep "^/fmu/out/" | wc -l)
```
This directly verifies MicroXRCEAgent ↔ FC connection health.

### 3. Motion Capture Tracking Node Timeout Fix
**What:** The `motion_capture_tracking_node` was timing out during startup because it was looking for the string `"Joined multicast group"` to signal successful initialization.

**Why:** After the OptiTrack PC rebooted and streaming was restarted, the node's initialization sequence changed slightly, and it printed `logClouds=0` before completing the multicast join.

**Hurdle:** Changed the search pattern from just `"Joined multicast group"` to `"logClouds\|Joined multicast"`, allowing the startup script to detect success earlier in the initialization chain.

**Solution:** Updated the monitor pattern in `startup-sequence.sh` line 183 to accept either output.

### 4. CRITICAL: MoCap Rigid Body Tracking Loss
**What:** The `mocap_px4_bridge` would hang silently when the OptiTrack Motive application lost track of the drone's rigid body. The bridge was running and publishing the `/fmu/in/vehicle_visual_odometry` topic, but with **zero publishers** on `/poses`.

**Why:** The bridge subscribes to `/poses` (published by `motion_capture_tracking_node`) and waits indefinitely for pose data containing the drone's rigid body name. When Motive loses tracking, `motion_capture_tracking_node` still runs but publishes empty pose arrays, causing the bridge to silently wait forever.

**Hurdle:** This is the most insidious failure mode: all ROS nodes appear "healthy" (running, publishing), but the system doesn't work because the expected rigid body data never arrives. Requires manual intervention at the OptiTrack PC to re-detect the rigid body.

**Solution:** 
1. Physically move the drone in the capture volume to force Motive to re-acquire tracking.
2. Verify the rigid body reappears in the Motive viewport.
3. Check `ros2 topic echo /poses` to confirm pose data is flowing before starting offboard missions.

### 5. Multi-Angle Collision Experiment Modular Refactoring
**What:** Extracted all data parsing, derivative filtering (Savitzky-Golay), plotting (top-down trajectories with vector SVG drones, battery profiles, velocity profiles), and statistical comparison modules out of the `experiments_analysis.ipynb` notebook into a dedicated package `experiments_analysis/` under `dev_logs/analysis/`.

**Why:** To prevent massive code duplication across multi-angle notebooks (0° to 90°, 5x passes each) and solve the Jupyter module caching bug once and for all via a robust reload guard. Also restores the strict aspect ratio and normalized 0.5m grid ticks.

**How it compares:** Previously, the notebook was a monolithic 700KB file containing heavy, duplicated plotting code that regularly threw import errors. Now, the notebook is a clean, parameter-driven frontend with only a single orchestrator `run(...)` function call per angle.

**Hurdle:** Jupyter caches imported modules in `sys.modules`, which previously locked down outdated package states.

**Solution:** Added a self-healing reload guard that clears the `experiments_analysis` cache dynamically in the notebook initialization cell, and fully validated the notebook headlessly with `nbconvert` (exit code 0).

### 6. Symmetrical 75° Column Collision Path Optimization
**What:** Optimized `exp_collision_75deg.py` to trace a strictly vertical straight-line sweep parallel to the Y-axis at a constant offset of $X = -0.186$ m, matching the exact CAD sketch and `/poses` column offset. 

**Why:** The original coordinate setup had an asymmetrical slope, which risked missing the column. The new path uses the takeoff pad ($X=0, Y=0$) for a safe vertical takeoff, shifts horizontally to the start of the sweep, flies a straight parallel line to guarantee a $3.4\text{ mm}$ physical overlap impact at the $Y = 0.186$ m column centerplane, shifts back to the center-line, and returns to takeoff coordinates safely.

**How it compares:** Previously, travel velocities were uniform; now it forces `1.0 m/s` rapid transit and recovery, leaving the constructor parameter `sweep_speed` to solely and safely control the active `WP2 -> WP3` impact segment.

---

## Technical Overview of Changes

### Files Modified or Created

1. **`dev_logs/analysis/experiments_analysis/`** (NEW analytical package)
   - `__init__.py`: Public API interface (`run`, `compare_all_angles`).
   - `exa_loader.py`: Re-engineered MCAP data ingestion and Pandas frame parser.
   - `exa_kinematics.py`: Savitzky-Golay numerical derivatives, waypoint arrival logging, and metric calculations (closest approach clearance, tracking error, average speed).
   - `exa_plot_trajectory.py`: rotated 2D spatial trajectory drawing using vector CAD drone graphics (`drone_top.svg`).
   - `exa_plot_kinematics.py`: Savitzky-Golay velocity profiles and dual Y-axis battery sag plots.
   - `exa_plot_statistics.py`: 2x2 boxplots comparing With vs. Without Cage side-by-side, and cross-angle progression trend line charts.
   - `exa_pipeline.py`: Master analysis pipeline coordinator.

2. **`experiments_analysis.ipynb`** (Modified)
   - Transformed from a heavy script-loaded notebook into a parameter-driven frontend dashboard using our new backend package.

3. **`drone_control/missions/exp_collision_75deg.py`** (Modified)
   - Renamed class to `ExpCollision75Deg` and mission title to `"75° Column Collision Loop"`.
   - Symmetrized the loop: Takeoff $\to$ `WP1(0.000, 1.200)` $\to$ `WP2(-0.186, 1.200)` $\to$ `WP3(-0.186, -1.200)` (sweep) $\to$ `WP4(0.000, -1.200)` $\to$ Loop.
   - Set transit, approach, and recovery speeds explicitly to `1.0 m/s` to keep horizontal commutes fast.
   - Restricted constructor-controlled `sweep_speed` exclusively to the active `WP2 -> WP3` sweep leg.

4. **`startup-sequence.sh`**
   - Removed hardcoded NTP fallback IP `192.168.74.2`
   - Added config read for OptiTrack server IP: `MOCAP_NTP=$(python3 -c "import json; print(json.load(open('/home/ws/config/drone_config.json'))['optitrack_server_ip'])")`
   - Changed `motion_capture_tracking_node` search pattern: `"logClouds\|Joined multicast"`
   - Added post-startup health diagnostic checking for `/fmu/out/*` topics and reporting connection status.

### Motive Application Configuration (Critical Settings)
Based on screenshot inspection of Motive 3.3.4.1:
- **Camera Frame Rate:** 360 Hz (default was 120 Hz; optimal for the cage setup)
- **Streaming Data Port:** 1511, Streaming Command Port: 1510
- **Multicast Address:** 239.255.42.99
- **Local Interface:** 192.168.74.3 (OptiTrack PC static IP)
- **Transmission Type:** Multicast
- **6 Cameras Active:** All enabled with 250 µs exposure, LED enabled
- **Streaming Rate:** ~980 B/frame, 358 Hz actual (measured at session time)
- **Residual (Tracking Quality):** 0.72 mm (acceptable; < 1 mm is good)

---

## Outcome

✅ **Startup script now reads all network config from `drone_config.json`—no more hardcoded IPs**
✅ **Health diagnostic clearly distinguishes FC stale connection from MoCap issues**
✅ **Motion capture tracking node succeeds on startup (search pattern fixed)**
✅ **Identified and documented the "MoCap rigid body loss" failure mode**
✅ **Modularized entire Jupyter analysis workflow into a clean backend package**
✅ **Restored strict aspect ratio and 0.5m scaled grids in trajectory plots**
✅ **Integrated self-healing reload guard to permanently solve Jupyter import caching conflicts**
✅ **Successfully verified headless notebook execution and TikZ file compilation (exit code 0)**
✅ **Symmetrized and validated the 75° Collision Sweep Mission path and speeds**
✅ **Startup script dynamic configuration and FC diagnostics verified**

---

## Learning Summary

1. **Thesis Signal Processing**: Differentiating raw MoCap signals requires a high-order Savitzky-Golay window to filter sensor jitter without shifting phase or lagging physical impact markers.
2. **Symmetrical Mission Trajectories**: Centering transit, approach, and recovery points around the takeoff pad origin ($X=0, Y=0$) allows safe, easily adjusted sweep offsets parallel to the Y-axis.
3. **Ghost ROS 2 Topics**: An active topic does not mean data is flowing. Checking publisher counts and raw pose arrays prevents silent bridge hangs during rigid-body dropouts.
4. **Thin-Notebook Architecture**: Parameterizing the notebook cells and offloading kinematics and graphing logic to a structured backend package enables immediate, repeatable analyses across hundreds of flight passes.

---

## Next Steps

1. **[HIGH PRIORITY] Before every startup session:**
   - Verify Motive is running and the drone's rigid body is tracked (visible in Motive viewport).
   - Check Motive streaming shows "980 B/frame" or similar (non-zero data rate).
   - Only then run `./startup-sequence.sh`.
2. Open `experiments_analysis.ipynb` in VS Code to review the rendered modular plots and comparative boxplots.
3. Progressively run and populate the collision sweep templates (75° to 0°) as experimental runs are gathered.
4. Implement deceleration ramping for smoother trajectory transitions.
5. Add automatic battery failsafe (voltage monitoring + land trigger).
