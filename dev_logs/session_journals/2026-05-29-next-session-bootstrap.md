# Next Session Bootstrap: 2026-05-29 — Symmetrical Cage Collision Telemetry & Comparative Thesis Plotting

Welcome! This bootstrap document is structured to help you instantly resume development and run the comparative analysis pipeline for Dorten's Master's Thesis experiments.

---

## 🎯 Primary Goals for Today
1. **Execute Comparative Flight Runs:** Run consecutive loops of the [exp_collision_75deg_v2.py](file:///home/dorten/pi_drone_sshfs/drone_control/missions/exp_collision_75deg_v2.py) mission under both `Rotating Cage` and `Fixed Cage` configurations to capture physical contact dynamics.
2. **Automate Segmentation & DB Caching:** Settle all new flight bags through the state-driven segmenter [mcap_event_segmenter.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/mcap_event_segmenter.py) to automatically cache metrics in SQLite.
3. **Verify Pipeline Integrity:** Ensure [exa_pipeline.py](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis/exa_pipeline.py) runs flawlessly over the new flights.
4. **Compile Comparative Boxplots:** Open [experiments_analysis.ipynb](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis.ipynb) to generate thesis-ready trajectory plots and 2x2 comparison statistics!

---

## 🛠️ Step-by-Step Execution Plan

### STEP 1: Live Flight Operations Protocol
Perform consecutive flight runs to rapidly expand the telemetry dataset:
1. **OptiTrack / Motive Verification Checklist:**
   - Open Motive GUI and verify `jake_drone_frame_01` (primary role in [drone_config.json](file:///home/dorten/pi_drone_sshfs/config/drone_config.json)) is actively tracked and streaming.
   - **Streaming panel health check:** Residual must be < 1.0mm, data rate ~358 Hz, payload size ~980 B/frame.
2. **Initialization Sequence:**
   Launch the system startup sequence (ensuring PX4 offboard link and Foxglove connections are active):
   ```bash
   ./startup-sequence.sh
   ```
3. **Execute 75° Collision Loop v2 (Rotating vs. Fixed):**
   - Run 3–5 loops under the `Rotating Cage` configuration.
   - Run 3–5 loops under the `Fixed Cage` configuration.
   - *Command:* In the flight control terminal, execute the `ExpCollision75DegV2` loop mission.

### STEP 2: Slicing and Auto-Repairing Telemetry
Once the flight is completed and the Pi has saved the raw MCAP bags inside `dev_logs/flights/`:
1. **Run the state-driven Event Segmenter:**
   ```bash
   python3 dev_logs/analysis/experiments_analysis/mcap_event_segmenter.py
   ```
   This will automatically scan all new flight bags, fix any truncation/corruption issues, parse active waypoint events from `/flight_director/active_waypoint`, and slice them into individual `-passXX.mcap` bags.

### STEP 3: Database Update and Web Inspection
1. **Execute the analysis pipeline:**
   ```bash
   python3 -m dev_logs.analysis.experiments_analysis.exa_pipeline
   ```
   This updates [collision_experiments.db](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/collision_experiments.db) idempotently.
2. **Generate the interactive DB HTML Inspector:**
   ```bash
   python3 dev_logs/analysis/experiments_analysis/generate_db_html.py
   ```
3. **Inspect details in the browser:**
   Open the beautiful dark-mode glassmorphic database dashboard:
   - [db_inspector.html](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/db_inspector.html)

### STEP 4: Render Thesis-Ready Visualizations
1. **Re-run the Analysis Notebook:**
   Execute all cells in [experiments_analysis.ipynb](file:///home/dorten/pi_drone_sshfs/dev_logs/analysis/experiments_analysis.ipynb) to output high-fidelity comparative figures:
   - 2x2 comparison boxplots (Rotating vs. Fixed Cage clearance and recovery times).
   - Multi-angle shock progression subplots.
   - Standalone LaTeX vector `.tikz` graphics!

---

## ⚠️ Known safety boundaries & tips
* > [!IMPORTANT]
  > **Shortened Runway Buffer:** The V2 mission uses a 10cm staging buffer to let the drone settle before entering the sweep. Keep transit speeds strictly at `0.30 m/s` to prevent any geofence boundary overshoots!
* > [!WARNING]
  > **DDS Serial Link Freeze:** Never terminate the micro-ROS XRCE Agent process once connected to the flight controller, as doing so freezes the PX4 serial buffer and requires a hard drone battery power cycle.
* > [!TIP]
  > **Battery Management:** Keep drone starting voltages above `16.0V` ($\ge 80\%$ capacity) before launching a loop sequence to prevent aggressive sag under collision impact thrust.
