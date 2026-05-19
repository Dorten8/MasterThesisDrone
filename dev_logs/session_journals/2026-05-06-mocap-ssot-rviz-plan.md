# Session Journal: 2026-05-06 — MoCap/PX4 Status + Next-Step Reset

## Where Are We in the Project
- The core data path is alive again: MoCap publishes `/poses`, and the bridge pushes updates to `/fmu/in/vehicle_visual_odometry` on the Pi.
- The thesis-critical gap now is not basic connectivity, but proving and documenting **how PX4 interprets MoCap as source of truth (SSoT)** for controlled flight.

## What We Worked On and Why

### 1. Re-validated bridge activity to PX4 input
- **What:** Confirmed live change on `/fmu/in/vehicle_visual_odometry` after earlier environment disruptions.
- **Why:** This protects confidence in the pipeline before spending time on higher-level control logic.
- **How it compares:** Better than prior uncertainty after restart issues; still short of full EKF acceptance proof.
- **Hurdle:** Available time was heavily reduced by monitor troubleshooting and TA duties.

### 2. Clarified frame-conversion discussion to revisit
- **What:** Captured that MoCap and PX4 frame semantics (ENU/NED handling and where transforms should be published) still need a focused architecture decision.
- **Why:** Without this, RViz interpretation and flight-command reasoning can drift from what PX4 actually uses.
- **How it compares:** We now have a concrete question set; implementation decision is intentionally deferred.
- **Hurdle:** Prior detailed session notes are not currently available.

### 3. Scoped RViz visualization approach for later execution
- **What:** Agreed direction: keep raw MoCap stream, keep PX4 odometry stream, and optionally add RViz-friendly ENU republish.
- **Why:** Separates debug visualization needs from PX4 estimator input requirements.
- **How it compares:** Cleaner than forcing one representation to satisfy both PX4 and RViz.
- **Hurdle:** RViz node/republisher was not implemented today.

## Technical Overview of Changes
- No production code change was made today.
- Operationally verified topic behavior:
  - Upstream MoCap topic: `/poses`
  - PX4 bridge input topic: `/fmu/in/vehicle_visual_odometry`
- Architecture note carried forward: `/dev/ttyAMA0` remains single-owner at runtime.

## Outcome
- ✅ **Deliverables**
  - Re-established confidence that bridge traffic reaches PX4 visual odometry input topic.
  - Consolidated next-step focus around RViz visibility, SSoT frame semantics, and command-loop testing.
- ⏳ **Not Done Yet**
  - RViz visualization from MoCap in a stable workflow.
  - Finalized SSoT frame publication/consumption strategy (MoCap ↔ PX4).
  - Iteration on `offboard_control.py` for command + acknowledgment test loop.

## Learning Summary
1. Topic activity proof (`/fmu/in/vehicle_visual_odometry`) is necessary but not sufficient; estimator acceptance must still be verified.
2. Visualization frames and estimator frames serve different consumers and should be kept explicit.
3. Progress can still be meaningful on constrained days when uncertainty is reduced and next decisions are made concrete.

## Next Steps
1. Get RViz working from MoCap data (initially in `world`/ENU for clean visual debugging).
2. Define and verify how PX4 takes MoCap as SSoT (frame semantics, transform location, estimator interpretation).
3. Extend `offboard_control.py` into a simple command/ack test path (e.g., arm/takeoff/hover/land primitives with acknowledgment visibility).
