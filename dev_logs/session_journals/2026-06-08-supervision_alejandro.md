### 1. Thesis Structure & Framing

Your supervisor emphasized moving from the general to the specific.

* **Introduction / General Context:** Start broader before diving into the fixed vs. rotating cage comparison. Explicitly discuss **cageless drones** first. Point out that standard drones cannot fly indoors safely without breaking upon impact. Frame your caged drone as the solution that allows for smooth, slow, and safe indoor flight.
* **Results / Experiments Section:** Keep the direct data analysis here. Explain what the specific graphs, plots, and metrics actually mean in the context of the experiment.
* **Discussion Section:** Keep this section open and general. This is where you hypothesize *why* the data looks the way it does (e.g., why a certain cage performed better at a certain angle).
* **Annex:** Push the massive, exhaustive raw datasets here, only keeping the most relevant plots in the main text.

### 2. Data Formatting & Outliers (To-Do List)

* **Capacity Drain Rate (Battery):** The current data is skewed (showing massive drain like 22.7% per minute, or 4-minute max flights). **Action:** Truncate the data. Strip out the idle/waiting time on the ground and only calculate the drain rate from the exact moment of commanded takeoff.
* **Voltage Drop:** The standard deviation is currently too high, and the data looks messy. **Action:** Revisit the voltage drop calculations to remove the noise.
* **Questionable Metrics:** You may need to drop or heavily caveat "Integrated Rotational Energy," "First Set Point Achieved," and "Max Analog Torque" if the calculations remain unclear or don't add value.

### 3. Metric Comparisons: Fixed Cage vs. Rotating Cage

Here is the breakdown of the physical performance based on your discussion.

| Metric | Better Performer | Observations & Hypotheses |
| --- | --- | --- |
| **Max Trajectory Deviation** | Fixed Cage | The rotating cage carries extra momentum/spin, which pushes the drone off its intended track. |
| **Path Spread** | Fixed Cage | Aligns with the trajectory deviation; the fixed cage keeps the drone closer to where it's supposed to be. |
| **Deceleration / Impact Acceleration** | Fixed Cage | Reduces impact acceleration by 24%. Normalizing the data to 1 shows a significant difference despite outliers. |
| **Recovery Area (Bounce Back)** | Fixed Cage (mostly) | At a 40–50° impact angle, the fixed cage has a smaller/better recovery area. *Hypothesis:* For straight-on impacts (0–30°), the rotating cage gets "rammed" because it doesn't know which way to deflect the energy, making it perform worse. |
| **Voltage Drop** | Fixed Cage | Experiences less voltage drop overall (though the data needs cleaning). |
| **Flight Time / Battery** | Fixed Cage | Anecdotally, you noted achieving longer flights and more loops with the fixed cage. |
| **Max Actuator Output** | Tie | Essentially the same; differences are within the margin of error. |
| **Commanded Motor Speed** | Tie | Same as above; no meaningful difference. |

### 4. Technical System Checks

* **MOCAP Tracking:** Despite concerns that a fast-rotating cage might mess with the cameras, the MOCAP successfully tracks the X, Y, and Z axes (roll, pitch, yaw) without dropping the pipeline.
* **PID Controller:** The drone flies slowly and smoothly. This is intentional, though you noted it could fly much faster if the PID wasn't tuned for this specific slow, controlled behavior.

---

# Session Journal: 2026-06-08 — MoCap Velocity Dropout Diagnosis & EKF Velocity Breakthrough

## Where Are We in the Project

Still in the **data analysis pipeline phase** — specifically, fixing the Fixed Cage velocity profile kinks that have been a persistent problem for months. The MoCap `/poses` topic drops to ~10 Hz during Fixed Cage flights (compared to ~120-240 Hz nominal), creating non-physical kinks in velocity and acceleration plots when differentiated. The rotating cage flights are clean.

Today we:
1. Diagnosed why no linear filter can fix this
2. Failed and rolled back one filtering attempt
3. Had a supervisor call where he suggested using PX4 EKF velocity instead
4. **Proved the EKF velocity approach works** — and it's spectacular

## What We Worked On and Why

### 1. Root-Cause Diagnosis of Fixed Cage Velocity Kinks

**What:** The Flight Kinetic Profile for Fixed Cage shows persistent non-physical kinks in the velocity trace. These come from MoCap dropouts (as low as ~10 Hz publish rate on `/poses`), which create slope discontinuities when linearly interpolated to 100 Hz for Savitzky-Golay differentiation.

**Why:** Every prior LLM and approach failed because the problem is fundamentally unsolvable with linear filters. The dropout artifacts sit at ~10 Hz (100 ms gap → half-cycle = 5-10 Hz). The real collision deceleration lasts ~250 ms → ~4 Hz. These are only **1.3 octaves apart** — no linear time-invariant filter has a transition band sharp enough to separate them without ringing.

**Failed Attempt 1 — Position Pre-Filtering:**
- Applied a 12 Hz Butterworth LPF to MoCap positions *before* SG differentiation
- Relaxed the velocity Butterworth from 4 Hz → 20 Hz
- **Result:** Way worse. The 12 Hz cutoff barely attenuated the 10 Hz kinks, while relaxing the velocity filter removed the only effective smoothing. **Fully reverted.**
- Documented in `kin_calculator.py` lines 108-118 as a disabled block with failure explanation.

### 2. Supervisor Suggestion: Use PX4 EKF Velocity

**What:** During the supervision call, Alejandro asked the critical question: *"But from the mocap poses or the px4 log poses?"* pointing out that PX4's EKF already fuses MoCap + IMU and computes its own velocity estimate.

**Why:** The PX4 EKF doesn't care about MoCap dropouts — it has high-rate IMU acceleration (100-250 Hz) to propagate velocity through the gaps. The result should be inherently smooth.

**How:** We extracted `velocity[0, 1, 2]` from the existing `/fmu/out/vehicle_odometry` messages (ROS 2 topic), which were already being parsed in `db_loader.py` but only for position. The velocity was already there — just unused. Added `vx_ekf_raw`, `vy_ekf_raw`, `vz_ekf_raw` columns to the `df_odom` DataFrame (purely additive change, lines 134-142).

### 3. EKF Velocity Comparison — the "Absolute Gold" Result

**What:** Created two new notebook cells in `experiments_analysis.ipynb`:
- **Single-flight EKF viewer** — 3×2 grid comparing MoCap SG velocity (left column) vs EKF velocity (right column) for one pass, plus a speed magnitude overlay
- **Dual comparison** — Fixed Cage vs Rotating Cage side-by-side (2×2 grid: speed + per-axis for each)

**Result:** The EKF velocity is **inherently smooth** — no dropout kinks at all. For Fixed Cage, the MoCap SG trace has visible kinks and the EKF trace is as clean as the Rotating Cage. For Rotating Cage, they agree closely, validating the coordinate alignment.

## Technical Overview of Changes

1. **[db_loader.py:134-142](db_loader.py#L134-L142)** — Added `vx_ekf_raw`, `vy_ekf_raw`, `vz_ekf_raw` to existing `df_odom` DataFrame. Zero impact on existing code — purely additive extraction from already-parsed ROS messages.

2. **[experiments_analysis.ipynb](dev_logs/analysis/experiments_analysis.ipynb)** — Two new code cells after the interactive viewer:
   - `🧭 EKF VELOCITY VIEWER` — `plot_ekf_vs_mocap_velocity()` function + Fixed Cage 45° example
   - `🧭 EKF DUAL COMPARISON` — `plot_dual_ekf_comparison()` running Fixed vs Rotating side-by-side

3. **[kin_calculator.py:108-118](kin_calculator.py#L108-L118)** — Commented-out position pre-filter block with failure explanation (disabled, not deleted).

## MoCap Velocity Downstream Impact — Complete Trace

The MoCap-derived velocity (`vx`, `vy`, `vz`, `speed`, `ax`, `ay`, `az`, `accel`) flows into:

| Thesis Artifact | Depends on MoCap Velocity? | What it uses |
|---|---|---|
| **Velocity Profile Figures** (Flight Kinetic Profile) | ✅ Yes | `vx`, `vy`, `vz`, `speed` |
| **Deceleration Improvement Claim** ("24.1% reduction") | ✅ Yes | `impact_accel` (2nd derivative of MoCap position) |
| **Impact Angle Calculation** | ✅ Yes | `vx`, `vy` direction vector |
| **Sweep Transit Speed** | ✅ Yes | `speed` → `avg_speed_wp2_wp3` metric |
| **Path Tracking / Clearance** | ❌ No | Position-based only |
| **IMU Structural Metrics** | ❌ No | Raw IMU data |
| **Motor RPM** | ❌ No | Actuator data |

The headline deceleration metric (24.1% improvement) depends on MoCap acceleration contaminated by dropout noise for Fixed Cage flights. The EKF velocity provides a clean alternative that could strengthen this result.

## Outcome

### ✅ Done
- Root cause of Fixed Cage velocity kinks fully understood and documented
- Position pre-filtering attempted, failed, and fully reverted (no damage to existing pipeline)
- EKF velocity extracted from existing vehicle_odometry messages
- Single + dual comparison plots created and verified (syntax OK)
- User confirmed the EKF velocity result is "absolute gold" and "this is what I needed the whole time"
- Full downstream dependency trace of MoCap velocity mapped
- **Plots backup created** — All 1,117 current plots (33 graphics + 1,084 per-pass capsule PNGs, 955 MB) saved to `dev_logs/analysis/plots_before_vel&acc_fix/` as a pre-EKF-integration baseline
- **copilot-instructions.md updated** with the EKF velocity breakthrough documentation, root cause findings, and updated priority order

### ⏳ Not Done Yet
- EKF velocity is **not yet integrated** into the main comparison pipeline (only viewable in the new notebook cells)
- `compute_flight_metrics()` still uses MoCap-derived `speed` and `accel` — EKF could replace these
- The dual EKF comparison plot is candidate thesis figure material for the "Methodology" or "Data Quality" section
- Deceleration-vs-battery plots still use MoCap-derived `impact_accel`

## Learning Summary

1. **Linear filters cannot separate signals 1.3 octaves apart.** This was the fundamental limitation — no Butterworth, elliptic, or Chebyshev filter can selectively remove 10 Hz dropout artifacts while preserving 4 Hz collision dynamics. The problem demanded a non-linear or model-based approach.
2. **The PX4 EKF is a sensor fusion engine, not just a position source.** Its velocity estimate (`vehicle_local_position.vx` / `vehicle_odometry.velocity[]`) is robust to MoCap dropouts because it propagates through gaps using IMU acceleration integration.
3. **The answer was in the data all along.** The velocity fields in `vehicle_odometry` ROS messages were already being parsed (for clock synchronization in `db_pipeline.py`) — nobody thought to use them as an alternative velocity source.
4. **"Check what the drone already computes"** is a powerful debugging heuristic — instead of trying to fix bad data in post-processing, use a better source that never had the problem.

## Next Steps

1. (Optional) Integrate EKF velocity into the main pipeline — replace MoCap-derived `speed` and `accel` in `compute_flight_metrics()` with EKF equivalents, or provide a configurable switch
2. Generate the dual EKF comparison figure as a thesis-ready graphic for the methodology section
3. Re-run the deceleration-vs-battery plots using EKF-based acceleration to see if the 24.1% improvement changes (likely strengthens)
4. Update the [experiments_analysis_skill.md](dev_logs/analysis/experiments_analysis_skill.md) with the EKF velocity solution for posterity