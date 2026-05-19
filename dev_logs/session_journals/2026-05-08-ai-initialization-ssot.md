# Session Journal: 2026-05-08 — AI Initialization & SSoT Reconfiguration

## Where Are We in the Project
We are at the initialization phase for the AI workflow, ensuring the AI agent correctly prioritizes its Single Source of Truth (SSoT) to be fully aligned with the project's ecosystem, Tutoring mode, and constraints before writing code.

## What We Worked On and Why
### 1. Reassigning the AI SSoT
- **What**: Updated the `GEMINI.md` rule file to point to `.github/copilot-instructions.md`.
- **Why**: To ensure the AI prioritizes the right SSoT rather than defaulting to `README.md`.
- **How it compares**: Previously, the AI was defaulting to `README.md`, which is good for general setup but misses critical Tutor mode rules, formatting guidelines, and the current session status.
- **Hurdle**: Recognizing the conflict between the user's intended SSoT and the one historically hardcoded in the AI instructions.

## Technical Overview of Changes
- Modified `GEMINI.md` to declare `.github/copilot-instructions.md` as the Primary Source of Truth.
- Read and internalized `.github/copilot-instructions.md` (Tutor persona, Fish shell, Mocap->PX4 architecture).
- Executed the end-of-day routine.

## Outcome
- ✅ Realigned AI context and SSoT pointing to `copilot-instructions.md`
- ⏳ Pending technical work: verifying RViz on laptop with Pi network.

## Learning Summary
1. The AI should prioritize `.github/copilot-instructions.md` over all other documentation to adhere strictly to the project's logic and the user's learning style.

## Next Steps
1. Verify RViz on laptop with Pi network (connect via SSHFS, confirm 30 Hz real-time updates in `drone_odometry.rviz`).
2. Confirm frame semantics (NED/ENU).
3. Extend `offboard_control.py` (waypoint loop/hover).
4. Test flight recording & replay.
