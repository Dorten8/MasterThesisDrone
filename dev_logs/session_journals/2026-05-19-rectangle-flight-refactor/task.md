# Rectangle Flight & Safety Refactor Checklist

- [x] Decouple Waypoint Math & Centralize State Transitions (`base_mission.py`)
- [x] Implement Global Keyboard Interrupt/Resume (`[ENTER]`)
- [x] Official `px4_ros_com` Arming Call (`param1=1.0`) Integration
- [x] Dynamic Menu Discovery for Mission scripts
- [x] Explicit Target Speeds per Waypoint
- [x] Restructure and Clean Up Repository Folders (Rationalize)
- [x] Consolidate Dev Log folders to `dev_logs/session_journals/`
- [x] Walk and Record Physical MoCap cage limits (`ghost_flight.py`)
- [x] Automated Geofence Calibration Utility (`calibrate_bounds.py`)
- [x] Multi-Dimensional Geofence enforcement (`flight_director.py`)
- [x] Validate Pre-Flight Safety Geofence Aborts (Nonsense Test)
- [ ] Execute Live Rectangle Flight Test
