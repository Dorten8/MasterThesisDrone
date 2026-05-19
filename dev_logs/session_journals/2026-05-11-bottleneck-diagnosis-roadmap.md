# Session Journal: 2026-05-11 — Bottleneck Diagnosis & Infrastructure Overhaul

## Where Are We in the Project
We are transitioning from "Maiden Flight" mode to "Rigorous Diagnostic" mode. After observing an uncontrolled climb acceleration at ~1.6m altitude, we have pivoted to establishing a solid data pipeline and hypothesis tracking system to isolate the cause (sensor fusion vs. control logic).

## What We Worked On and Why

### 1. Infrastructure Overhaul
- **What**: Created a centralized `dev_logs/` folder and updated all recording tools.
- **Why**: To ensure flight data, logs, and documentation are organized and easily accessible for the Master Thesis.
- **Hurdle**: ROS 2 bag files were previously scattered and used the older `.db3` format.
- **Outcome**: All data is now in `dev_logs/flights/` using the robust **MCAP** format.

### 2. Flying Bottlenecks Roadmap
- **What**: Created [flying_bottlenecks.md](file:///home/dorten/pi_drone_sshfs/dev_logs/flying_bottlenecks.md) and a formal [Implementation Plan](file:///home/dorten/.gemini/antigravity/brain/d532c87f-48bc-4839-8a28-c6d53164b0bb/implementation_plan.md).
- **Why**: To track hypotheses like Mocap dropouts, SSoT fusion switching, and axis orientation issues.
- **Outcome**: A clear set of executable steps for the next few days of testing.

### 3. Hardware Maintenance & SD Card Migration
- **What**: Replaced a broken motor mount (3D printed) and migrated PX4 logs to a new 32GB SD card.
- **Why**: Hardware stability is paramount for safe testing, and the 32GB card provides more headroom for high-rate logging.
- **Hurdle**: Found that PX4 did not record logs for today's flights, likely due to `SDLOG_MODE` settings.

## Technical Overview of Changes
- **Updated**: [record_flight_bag.sh](file:///home/dorten/pi_drone_sshfs/dev_logs/record_flight_bag.sh) (Added `-s mcap` and standardized paths).
- **Modified**: [.github/copilot-instructions.md](file:///home/dorten/pi_drone_sshfs/.github/copilot-instructions.md) (Updated session status).
- **Moved**: All flight bag directories into `dev_logs/flights/`.

## Outcome
- ✅ **Infrastructure**: Workspace is organized and documentation-ready.
- ✅ **Hardware**: Drone frame is repaired and SD card is upgraded.
- ⏳ **Diagnostics**: Initial analysis confirms missing PX4 logs; matching strategy developed.

## Learning Summary
1. **MCAP Superiority**: Noticed that MCAP is much better for thesis data as it includes schemas and is better for Foxglove/Jupyter.
2. **Logging Constraints**: Realized PX4 won't log without specific parameters (`SDLOG_MODE`) or successful arming durations.
3. **Clock Sensitivity**: Discovered the 2-hour offset between system time and flight names.

## Next Steps
1. Set `SDLOG_MODE=1` in PX4.
2. Create `sync_time.sh` for Pi-Mocap clock alignment.
3. Attempt coordinate-based matching of today's bags with older PX4 logs.
4. Execute the 1m hover benchmark test.
