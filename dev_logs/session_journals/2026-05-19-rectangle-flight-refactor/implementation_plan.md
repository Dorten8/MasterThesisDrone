# Implementation Plan: Repository Rationalization & MoCap Boundary Calibration

This plan outlines two key workflows to prepare for our dynamic flights:
1. **Repository Rationalization (Clean Up)**: Organizing and clean-naming files, archiving legacy draft scripts, and establishing a persistent, Git-tracked `dev_logs/session_journals/` folder so our AI lessons are structured and easy to pick up later.
2. **MoCap Boundary Recording & Integration**: Walking the cage bounds with `ghost_flight.py`, automatically calibrating geofences in `flight_director.py` from the CSV data, testing the nonsense geofencing checker, and executing our maiden rectangle flight.

---

## 📂 Phase 1: Repository Rationalization & Dev Log folder

To keep our high-latency SSHFS connection fast and prevent editor clutter, we will restructure the workspace as follows:

```
pi_drone_sshfs/
├── config/
│   ├── drone_config.json
│   └── mocap_bounds.json                 <-- [NEW] Automatically saved cage bounds
├── dev_logs/
│   ├── analysis/                         <-- [NEW] Diagnostic/jerk tests
│   │   ├── analyze_deep_dissect.py
│   │   └── analyze_jerk_test.py
│   ├── paths/
│   │   └── ghost_path_latest.csv
│   ├── session_journals/                 <-- [NEW] Consolidates all session files
│   │   ├── 2026-05-18-coordinate-alignment.md
│   │   ├── 2026-05-19-rectangle-flight-refactor/
│   │   │   ├── task.md                   <-- Copy of our active checklist
│   │   │   ├── implementation_plan.md    <-- Copy of this plan
│   │   │   └── walkthrough.md            <-- Copy of final walkthrough
│   │   └── ... (existing journal markdown files)
│   ├── flying_bottlenecks.md
│   ├── ghost_flight.py                   <-- Boundary & playback recorder
│   └── record_flight_bag.sh
├── drone_control/
│   ├── manual_control/                   <-- [NEW] Consolidates manual utilities
│   │   ├── xbox_controller.py
│   │   ├── xbox_controller_reader.py
│   │   └── mavlink_xbox_controller.py
│   ├── utils/                            <-- [NEW] Helper scripts
│   │   ├── start_xrce_agent.sh
│   │   └── mavlink_command_formatter.py
│   ├── missions/
│   │   ├── base_mission.py
│   │   ├── hover_test.py
│   │   └── rectangle_flight_relative_pos.py
│   ├── flight_director.py
│   ├── flight_recorder.py
│   └── emergency_kill.py
└── [DELETED] drone_control/hover_test_initial.py  <-- [ARCHIVED] fully obsolete
```

### Plan for Dev Log Consolidation
All tasks, walkthroughs, and plans we deal with will be cleanly archived into:
`dev_logs/session_journals/YYYY-MM-DD-topic/`
This makes it trivial for you to pick up exactly where we left off in subsequent sessions, keep your Notion perfectly in sync, and track all changes via Git.

---

## 🛸 Phase 2: MoCap Boundary Calibration & Rectangle Flight

Here is the exact step-by-step workflow we will use to map the room and verify the geofencing pre-checker.

### Step 1: Record the Boundary
1. You will start the `Ghost Flight Recorder` on the Pi:
   ```fish
   python3 dev_logs/ghost_flight.py
   ```
2. Pick up the drone and walk it in a clean box matching the absolute physical boundaries of your flight cage, raising and lowering it to define maximum safe height and depth.
3. Stop recording to save the boundary path to `dev_logs/paths/ghost_path_latest.csv`.

### Step 2: Calibrate the Geofence Bounds
We will write a simple utility script, `dev_logs/analysis/calibrate_bounds.py`, that:
- Reads the generated `ghost_path_latest.csv`.
- Computes the minimum and maximum X, Y, and Z.
- Appends a safety buffer (e.g. subtract `0.2m` inside the box for safety).
- Saves the parsed bounds to `config/mocap_bounds.json` as:
  ```json
  {
    "x_min": -1.2,
    "x_max": 1.2,
    "y_min": -1.2,
    "y_max": 1.2,
    "z_max": 1.8
  }
  ```
- We will modify `flight_director.py` to dynamically load `config/mocap_bounds.json` on startup instead of using hardcoded limits.

### Step 3: Test nonsense geofencing checker (Pre-flight safety)
1. We will load `RectangleFlightRelativePos` with a nonsense value (e.g. `relative_pos_x = 4.0m`).
2. Run the `FlightDirector`:
   ```fish
   python3 drone_control/flight_director.py
   ```
3. Press `[ A ]` to Arm and `[ T ]` to Take Off.
4. Verify that the Flight Director halts immediately and prints:
   `[FATAL] GEOFENCE PRE-CHECK FAILED! Waypoint 1 (4.00, 0.00, 0.50) violates geofence!`
   And refuses to arm/take off.

### Step 4: Execute the Dynamic Rectangle Flight
1. Reset `relative_pos_x = 0.8m`.
2. Arm and Take Off.
3. Verify that the drone flies the rectangle beautifully and stops at its final hover point, waiting for your manual confirmation.

---

## Open Questions

* **Safety Margin**: Is a `20cm (0.2m)` buffer inside the recorded boundaries comfortable for your cage, or would you prefer a tighter/looser safety buffer when parsing the boundaries?
