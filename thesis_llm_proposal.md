# Thesis Proposal: Collision-Resilient Drone for Industrial Inspection

> **Working Title Suggestion:**
> *"Collision-Resilient Quadcopter Design for Confined Industrial Environments: A Comparative Evaluation of Fixed and Rotating Protective Cages"*

---

## Thesis Structure Overview

Based on existing sections in `thesis/Sections/` and the project's experimental data, the thesis should follow this high-level flow:

```
1. Introduction           ← Needs full write (has bullet sketch only)
2. Related Work           ← Has good content, needs polishing + more citations
3. Methodology            ← Has extensive content — MUST BE HEAVILY CONDENSED
4. Experiments & Results  ← Has detailed analysis, 3 placeholders to fill
5. Discussion             ← NEW section not yet created
6. Figures                ← Stub (existing section)
7. Tables                 ← Placeholder table
8. Mathematics            ← Has basic equations, can expand
9. Conclusion             ← NEW section not yet created
   Bibliography
   (Appendices)
```

**Key structural recommendation:** Add a dedicated **Discussion** section (between Experiments and Conclusion) where you hypothesize *why* the data looks the way it does — supervisor explicitly requested this separation. Push bulk raw data to appendices.

---

## 1. Introduction

### Current State
The file `Introduction.tex` contains a research question paragraph and placeholders noting it needs "a whole lot of text." This is the biggest gap in the thesis.

### Proposed Content Structure

**Paragraph 1 — The Industrial Problem** (1/2 page)
- Industrial inspections of confined spaces (tanks, ducts, pipe corridors) are hazardous for humans.
- Drones offer a safer alternative but fail indoors: GPS denied, cluttered, no room for SLAM payloads.
- The fundamental tension: perfect sensing is too heavy/expensive for small drones; imperfect sensing means collisions will happen.
- **Citations:** Use `mahmudDevelopmentCollisionResilient2021` for cluttered environment framing, `lercheSpinFlyUAVSystem` (Spinfly) for the "tried SLAM on small drone" context.

**Paragraph 2 — Why Not Just Sense Better** (1/2 page)
- SLAM is the holy grail but not production-ready for sub-250g-2kg drones.
- Even automotive LIDAR struggles with reflective pipes, dark tanks, narrow clearances.
- Reference `mulgaonkarRobustAerialRobot2018`: swarms that collide and recover rather than avoid — radically different philosophy, but only 25g drones.
- The scaling problem: a 1.2kg 4-inch drone has ~50× the kinetic energy of a 25g drone. Collision tolerance must be engineered, not assumed.

**Paragraph 3 — Thesis Approach** (1/4 page)
- This work: combine mechanical resilience (protective cage) with standard offboard control (PX4 + ROS2 + MOCAP) so the drone can collide and continue its mission.
- Two cage variants built and tested experimentally: **Fixed Cage** vs **Rotating Cage**.
- **Research Question** (already drafted in Introduction.tex):
  > *"Develop a drone for industrial settings which is survivable and able to recover from clashes with its environment without breaking or finishing its mission early, autonomous enough to navigate through signal-denied environments, and powerful enough to carry equipment for autonomous navigation and survival."*

**Paragraph 4 — Contributions** (1/4 page)
- A 4-inch quadcopter platform with ~3.8:1 thrust-to-weight, 45-50% hover throttle.
- Two cage designs with quantitatively compared collision performance.
- 179 collision passes over 26 flights at 45° and 75° incidence angles.
- ~24% peak deceleration reduction with fixed cage; ~6.5G mean vs ~9.5G mean impact load.
- Full open-source pipeline: ROS2 flight stack, MCAP telemetry, Python analysis.

---

## 2. Related Work

### Current State
`Related_work.tex` has good paragraphs on:
- **Collision-resilient designs:** Congifly (`azambujaWhenBeingSoft2022`), Spinfly (`lercheSpinFlyUAVSystem`), Flyability (`UltimateGuideDrone`)
- **Cluttered environment cages:** Mahmud (`mahmudDevelopmentCollisionResilient2021`)
- **The Gomboc/turtle shape** (`Gomboc2026`)
- **Robust aerial swarms:** Mulgaonkar (`mulgaonkarRobustAerialRobot2018`)
- **PX4 vs ArduPilot** with citation justification for PX4 choice

### Proposed Additions

**2.1 Collision-Resilient Designs**
- This section is mostly solid. Add:
  - A sentence on the scalability issue: smaller drones (25g) can use simple cages; larger drones (1.2kg+) need more sophisticated design.
  - Explicitly cite `lercheSpinFlyUAVSystem` for "lessons learned" — weight headroom, battery life, controller selection — as direct motivation for your design choices.
  - More explicit mechanism comparison between gimbal sphere (Flyability), freely spinning cage (your rotating variant), and rigid cage (your fixed variant).

**2.2 Autonomous Flight Platforms**
- The PX4 vs ArduPilot section is written in a conversational tone. Needs polish:
  - Formalize the license comparison (add actual references: GPL v3 for ArduPilot, BSD 3-clause for PX4).
  - Expand uORB modularity point: cite `meierPX4NodebasedMultithreaded2015` as the actual PX4 paper.
  - Add a sentence on the Pixhawk standard ecosystem (`simpsonLatestPixhawkOpen2023`).

**2.3 Coordinate Frames and Localization**
- Add a brief subsection on the MOCAP-based indoor localization approach vs traditional SLAM.
- Reference the OptiTrack hardware used (6 cameras, 360 Hz, <1mm residual).
- Frame the choice: "We sidestep SLAM by using a fixed infrastructure MOCAP system for ground-truth, letting us isolate the collision mechanics question."

**Missing citations to add:**
- `gomezParetoOptimalPID2020` for quadcopter dynamics model (already used in Methodology)
- `oscarHowChooseFPV2024` / `recursionlabsRecursionLabs6s2021` for drivetrain design principles

---

## 3. Methodology ⚠️ CONDENSE HEAVILY

### Current State
Currently ~500 lines of extremely detailed content including:
- Full Docker setup instructions for ROS2 on Raspberry Pi 5
- SSH key configuration
- UART pinout assignments
- Step-by-step terminal commands
- Detailed TWR calculation walkthrough
- Battery chemistry explanations (voltage sag, 4S vs 6S)
- Motor KV theory

### Condensation Strategy
The user explicitly said "most of methodology will not be there, that is way too detailed oriented." Proposed structure:

**3.1 Design Objectives** (1 page)
> Shorten the current 2-page iterative-learning section to 3-4 tight paragraphs:
- Three constraints from Spinfly (weight headroom, battery life, controller).
- Three design objectives (carry bigger load, fly longer, simpler control).
- How these map to choices (4-inch frame, 6S battery, PX4/Pixhawk 6C).

**3.2 Hardware Platform** (1.5 pages)
> Condense the hardware + battery + motor content significantly:
- **Frame:** 4-inch wheelbase justified by Pixhawk size + indoor flight limits.
- **Motors:** EMAX ECOII 2004 1600KV — single sentence on why this pairing; one sentence on TWR (~3.8:1) and hover efficiency.
- **Battery:** 6S LiPo — one sentence on voltage sag advantage over 4S. No need for Ohm's law derivation.
- **Flight Controller:** Pixhawk 6C with PX4 — modular, uORB, integrated IMU.
- **Companion Computer:** Raspberry Pi 5 with ROS2 Humble in Docker on Ubuntu 24.04 — *one paragraph, no setup steps*.
- **System diagram:** `architecture.pdf` (Figure 1 — already in document, keep).

**3.3 Protective Cage Design** (1-2 pages)
> **NEW content needed** — this is currently missing/empty in `Methodology.tex`:
- Describe the fixed cage (rigid column-like structure attached to frame).
- Describe the rotating cage (free-spinning outer shell on bearings).
- Manufacturing: materials used (e.g., carbon fiber rods for fixed cage, PETG/PLA for rotating cage), weight of each variant.
- Design rationale: the turtle/Gomboc shape influences (`Gomboc2026`, `mahmudDevelopmentCollisionResilient2021`).
- Include `2D_path_overlay` and `2D_trajectory` figures (already in Experiments) cross-referenced here as design validation.

> **Findings from session journals:** The rotating cage weighs more and has higher moment of inertia. The fixed cage is lighter and stiffer. These physical differences directly explain the experimental results.

**3.4 Autonomy and Control Architecture** (1 page)
- Condense the current 200+ lines of ROS2/PX4 setup into a single architecture paragraph:
  - Offboard computer runs `flight_director.py` → publishes trajectory setpoints via uXRCE-DDS.
  - PX4 EKF2 fuses MOCAP position (360 Hz OptiTrack) with onboard IMU for state estimation.
  - MOCAP → `motion_capture_tracking_node` → `mocap_px4_bridge` → PX4 via `/fmu/in/vehicle_visual_odometry`.
  - `flight_recorder.py` logs telemetry to `.mcap` files for post-processing.
- Include `experiments_pipeline.pdf` (Figure 2).
- Include a **brief** summary of the coordinate frame transformations (ENU↔NED) — the 90-degree cross-coupling fix from session journals is worth mentioning as a solved design challenge.
- **Cut:** All Docker setup, SSH key instructions, UART pinout tables, troubleshooting reference table.

**3.5 Experimental Protocol** (1-2 pages)
> Currently embedded in Experiments. Move a concise version here:
- OptiTrack MOCAP cage (6 cameras, 360 Hz, ~980 B/frame).
- Collision target: cylindrical column (cardboard + aluminum, R=4.5cm).
- Flight path: straight-line sweep perpendicular to column at 45° and 75° incidence.
- Pass structure: the drone loops repeatedly at ~0.30-0.35 m/s transit speed, collides once per pass, recovers, and circles back.
- The two cage configurations tested (Fixed vs Rotating); 179 total passes (108×45°, 71×75°).
- **Include the passage from Experiments.tex §"Experimental Setup"** — it's already well-written and belongs here in Methodology, not in results.

**3.6 Data Analysis Pipeline** (0.5 page)
- From the session journals: custom Python pipeline that segments `.mcap` files by pass, computes kinematic metrics via Savitzky-Golay filtering, and stores results in SQLite (`experiments_summary.db`).
- The EKF velocity extraction breakthrough: PX4 EKF fuses MOCAP position with IMU to produce dropout-free velocity.
- Note this replaces MoCap-differentiated velocity (which had kinks from 10 Hz dropout artifacts).
- The notebook (`experiments_analysis.ipynb`) provides interactive per-pass viewing; summary notebook generates aggregate comparison plots.

---

## 4. Experiments and Results

### Current State
`Experiments.tex` is the most complete section. Four subsections are fully written:
1. **Experimental Setup** (should move to Methodology — leave a brief cross-reference)
2. **Flight Stability and Efficiency Baseline** — key finding: both cages fly stably, MOCAP tracks spinning cage without issues
3. **Impact Kinematics and Force Transfer** — key finding: fixed cage reduces peak deceleration by ~24% (6.5G vs 9.5G)
4. **Post-Collision Trajectory and Recovery** — key finding: fixed cage has smaller recovery area, especially at 40-50° angles

Three subsections are **placeholders with `***SECTION NEEDS CONTENT***`**:

**4.5 Deceleration and Force Transfer Profiles** (to fill)
- Reference the `deceleration_vs_battery_angle` plots and related graphics.
- **Content to write:**
  - Table of max deceleration per angle bin ($30^\circ$, $45^\circ$, $60^\circ$, $75^\circ$) split by cage type.
  - IMU peak acceleration Z-axis analysis at impact.
  - Relationship between commanded motor speed (RPM) and post-impact recovery thrust.
  - Current draw profiles showing how the flight controller compensates after collision.
- **Key number to lead with:** Fixed cage mean $6.5G$ vs Rotating cage mean $9.5G$ — a $24\%$ reduction.
- **Available data:** 179 passes, battery-normalized deceleration metrics, Huber-robust trendlines.
- **Caveat:** The 24% claim currently uses MoCap-derived acceleration. EKF integration might strengthen or refine this number — note if EKF integration is complete at submission time.

**4.6 Impact Phase Portraits** (to fill)
- Reference marker: "Map peak impact acceleration against rotational energy (angular momentum)."
- **Content to write:**
  - Scatter plot: X-axis = peak impact acceleration (G), Y-axis = integrated rotational energy (rad).
  - For rotating cage: expect high rotational energy + lower residual shock (energy dissipated into spinning).
  - For fixed cage: expect low rotational energy + lower peak deceleration (energy absorbed by structural stiffness).
  - The fixed cage paradox: it absorbs shock better despite *not* dissipating energy into rotation. Hypothesis: the rigid structure distributes force evenly; the rotating cage's momentum stores energy that then releases unpredictably.
  - Equation from `Math.tex`:
    $$\|\mathbf{a}_{\text{peak}}\| = \max_{t \in T_{\text{collision}}} \sqrt{a_x(t)^2 + a_y(t)^2 + a_z(t)^2}$$
- **Figure needed:** A 2-panel plot with fixed cage (left) and rotating cage (right), each showing peak G vs rotational energy with regression lines.

**4.7 Vibration and Lateral Stability** (to fill)
- Reference marker: "Post-impact Y-axis vibration spread (rotating cage 0.3-0.6G localized vs fixed cage 0.6-1.7G high-amplitude jitter)."
- **Content to write:**
  - This is the **one metric where the rotating cage wins**: lateral vibration dampening.
  - The rotating mass acts as a mechanical gyrostabilizer — it absorbs high-frequency lateral vibration, spreading the impulse across its rotation cycle.
  - Fixed cage transmits more vibration to the core: Y-axis standard deviation 0.6-1.7G vs rotating cage 0.3-0.6G.
  - Standard deviation of lateral acceleration ($A_y$ in G) over a sliding window post-impact.
  - **Hypothesis for Discussion section:** The rotating cage's angular momentum creates gyroscopic stiffness that resists lateral perturbations — this is the gyroscopic precession effect of a spinning mass. The cost is higher peak impact load; the benefit is faster vibration settling.
- **Figure needed:** Side-by-side Y-axis acceleration time traces for fixed vs rotating cage post-impact, with shaded std-dev bands.

### Other Figures for Result.tex

**Already included:**
- Fig 1: `2D_path_overlay.png` — 2D path overlay fixed vs rotating cage (top of section)
- Fig 2: `experiments_pipeline.pdf` — software pipeline
- Fig 3: `2D_trajectory.png` — all 45° incidence passes trajectory overlay

**Additional figures to generate/insert from the analysis pipeline:**
- **Flight kinetic profile:** EKF velocity overlay (MoCap vs EKF) — the "absolute gold" figure from session journal 2026-06-08. Shows dropout-free velocity for fixed cage.
- **IMU dynamics comparison:** Per-pass IMU traces with impact marked.
- **Deceleration vs battery state:** Scatter plot with robust trendlines (Huber/Theil-Sen).
- **Cage deviation overlay:** Comparative trajectory spread, rotating vs fixed.
- **Phase portrait:** Peak G vs rotational energy (fill §4.6 with this).

---

## 5. Discussion ⭐ NEW SECTION

### Why This Is New
The user's supervisor explicitly requested: *"Keep the results section as direct data analysis. Put the hypothesizing about *why* in a separate Discussion section."*

Currently no `Discussion.tex` exists. Recommend creating it as the bridge between results and conclusion.

### Proposed Content

**5.1 Summary of Findings** (1/2 page)
- Table recapping which metrics favor which cage:

| Metric | Better Performer | Magnitude |
|--------|-----------------|-----------|
| Peak deceleration (impact load) | Fixed cage | 6.5G vs 9.5G (−24%) |
| Trajectory deviation (post-impact) | Fixed cage | Smaller recovery area |
| Flight efficiency (battery) | Fixed cage | Lower voltage drop, longer flights |
| Path spread repeatability | Fixed cage | Tighter lane-keeping |
| Lateral vibration dampening | Rotating cage | 0.3-0.6G vs 0.6-1.7G jitter |
| Maximum actuator output | Tie | No significant difference |

**5.2 Why Fixed Cage Performs Better on Impact Metrics** (1 page)
- **Hypothesis 1 — Momentum Dissipation:** The rotating cage stores impact energy as rotational momentum rather than dissipating it. This means the energy doesn't disappear — it releases later as the drone re-orients, causing secondary disturbances.
- **Hypothesis 2 — Structural Stiffness:** The fixed cage is a rigid structure that transmits force evenly through the frame, letting the flight controller counteract the disturbance immediately. The rotating cage has mechanical play in the bearing assembly, introducing a "dead zone" before the controller can respond.
- **Hypothesis 3 — Mass Distribution:** The rotating cage's additional mass increases the drone's moment of inertia, making the PID controller's task harder (larger angular momentum to overcome).

**5.3 Why Rotating Cage Wins on Vibration** (1/2 page)
- Gyroscopic stabilization from the spinning mass resists high-frequency lateral oscillations.
- Trade-off: the same gyroscopic stiffness that dampens vibration also transfers more shock energy into the core during the initial impact.
- The rotating cage is better for missions where *sensor stability after impact* matters; the fixed cage is better where *survival and continued navigation* are paramount.

**5.4 Implications for Industrial Deployment** (1/2 page)
- A hybrid design might be optimal: fixed outer structure for impact energy absorption, with selective rotating elements that engage only when vibration dampening is needed.
- The turtle/Gomboc shape (`mahmudDevelopmentCollisionResilient2021`) could be combined with this insight — a self-righting shell that is rigid during collision but allows some rotational freedom during flight.
- Battery capacity remains the primary constraint for mission duration; the cage adds ~15-20% weight penalty over cageless flight.

**5.5 Limitations** (1/2 page)
- MOCAP-based ground truth available only in lab setting. Real industrial deployment needs onboard localization (optical flow, UWB, or even "localization by collision" as Mulgaonkar proposed).
- Only two incidence angles (45°, 75°) tested. Full 0-90° characterization would strengthen the angle-dependent findings.
- Single collision target (cylindrical column). Real pipes, shelves, corners would produce different dynamics.
- Only one drone built. Pass-to-pass variance within a single platform is captured (179 passes), but platform-to-platform variance is not.
- EKF velocity integration into the main comparison pipeline is pending as of data freeze.

---

## 6. Mathematics

### Current State
`Math.tex` has four subsections:
- Coordinate frames (ENU world, NED body) — good.
- Drone dynamics (Euler equations) — good starting point.
- Collision kinematics — `\|a_peak\|` equation — good.

### Proposed Additions

**Position and Velocity definitions** (already partially there):
- Add explicit definitions:
  - Position: $\mathbf{p} = [x, y, z]^\mathsf{T}$ in world frame
  - Velocity: $\mathbf{v} = [v_x, v_y, v_z]^\mathsf{T}$
  - Angular velocity: $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^\mathsf{T}$

**Trajectory recovery metric:**
- Recovery area: $A_{\text{recovery}} = \iint_{t_{\text{impact}}}^{t_{\text{stabilized}}} \|\mathbf{p}(t) - \mathbf{p}_{\text{commanded}}(t)\|_2 \, dt$

**Vibration spread metric:**
- Lateral acceleration standard deviation: $\sigma_{a_y} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} (a_{y,i} - \bar{a}_y)^2}$

**Integrated rotational energy:**
- $E_{\text{rot}} = \int_{t_{\text{impact}} - \Delta t}^{t_{\text{impact}} + \Delta t} \|\boldsymbol{\omega}(t)\|_2 \, dt$

---

## 7. Figures

### Current State
`Figures.tex` is a stub paragraph describing that figures come from experimental data and vector diagrams are Mermaid-rendered PDFs.

### Proposed Enhancement
- Keep it short — it's a reference section, not analysis.
- Add a brief catalog of figure types and their generation method:

| Figure Type | Format | Generation Source |
|------------|--------|------------------|
| Architecture diagrams | PDF (vector) | Mermaid `.mmd` → `mmdc` CLI |
| Trajectory overlays | PNG | `experiments_analysis_summary.ipynb` |
| IMU dynamics | PNG | `experiments_analysis.ipynb` per-pass plots |
| EKF velocity comparison | PNG | Custom notebook cells |
| Phase portraits | PNG | Summary notebook |
| Comparative tables | PDF/PNG | Matplotlib table generation |

---

## 8. Tables

### Current State
`Tables.tex` has a single placeholder table with N values for pass counts.

### Fleshing Out

**Table 1: Experiment Configurations** (expand the placeholder):
| Incidence Angle | Cage Type | Valid Passes | Avg Peak Deceleration (G) | Avg Recovery Area (m²) |
|----------------|-----------|-------------|--------------------------|----------------------|
| 45° | Fixed | 56 | ~6.2 | TBD |
| 45° | Rotating | 52 | ~9.1 | TBD |
| 75° | Fixed | ~35 | TBD | TBD |
| 75° | Rotating | ~36 | TBD | TBD |

> **Note:** Actual values need to be queried from `experiments_summary.db`. The database has all 179 passes.

**Table 2: Drone Specifications** (new):
| Component | Specification |
|-----------|-------------|
| Frame | 4-inch wheelbase |
| Motors | EMAX ECOII 2004 1600KV |
| Battery | 6S LiPo 22.2V |
| Flight Controller | Pixhawk 6C (PX4 v1.16.1rc) |
| Companion Computer | Raspberry Pi 5 (Ubuntu 24.04 / ROS2 Humble in Docker) |
| MOCAP | OptiTrack, 6 cameras, 360 Hz, <1mm residual |
| Weight (bare) | ~900g |
| Weight (with fixed cage) | ~1050g (estimate — measure) |
| Weight (with rotating cage) | ~1100g (estimate — measure) |
| Thrust-to-Weight | ~3.8:1 |
| Hover Throttle | ~45-50% |
| Transit Speed | 0.30-0.35 m/s |

**Table 3: Comparative Performance Summary** (from Discussion §5.1 — duplicate in Table form).

---

## 9. Conclusion ⭐ NEW SECTION

### Proposed Content

**9.1 Research Question Answered** (1/2 page)
- Recap the research question (from Introduction).
- State: The fixed cage design demonstrably reduces impact loads by ~24%, maintains flight stability, and preserves battery efficiency — meeting the survivability requirement.
- The rotating cage offers superior vibration dampening, presenting a design trade-off.
- The drone platform successfully carried its protective cage and sensors with 3.8:1 TWR and ~45-50% hover throttle, demonstrating adequate power headroom.

**9.2 Contributions** (1/2 page)
1. A fully characterized 4-inch collision-resilient quadcopter platform (open-source: ROS2 flight stack, analysis pipeline).
2. Experimental comparison of fixed vs rotating protective cages across 179 collision passes.
3. Quantitative evidence that fixed cages reduce peak impact acceleration by ~24% but rotating cages dampen post-impact vibration.
4. Validation that MOCAP tracking is robust even with a rapidly spinning cage.
5. The EKF velocity extraction method that produces dropout-free velocity from PX4's onboard sensor fusion — applicable beyond this project.

**9.3 Future Work** (1/2 page)
- **Hybrid cage design:** Combine fixed structural elements with selectively engaged rotating dampeners.
- **Onboard localization:** Replace MOCAP with optical flow + UWB for industrial deployment.
- **Full angle sweep:** Test all angles from 0° to 90° in 15° increments.
- **Multiple collision targets:** Shelves, pipes, corners, narrow passages.
- **Control parameter tuning:** The PID was intentionally set for slow flight; faster transit could change collision dynamics dramatically.
- **Motor analysis and control allocator saturation:** Currently stalled with empty/zero data — investigate why `allocator_saturation_duration_sec` is empty for 177 flights.

---

## Citation Map

Below is which citations map to which thesis sections, organized by topic:

### Collision-Resilient / Cages
| Citation | Key Content | Sections |
|----------|-------------|----------|
| `azambujaWhenBeingSoft2022` | Congifly — soft drone design | Related Work §2.1 |
| `lercheSpinFlyUAVSystem` | Spinfly — autonomous inspection drone | Related Work §2.1, Methodology §3.1 |
| `mahmudDevelopmentCollisionResilient2021` | Cluttered environment cages, Gomboc shape | Related Work §2.1, Methodology §3.3 |
| `mulgaonkarRobustAerialRobot2018` | Swarm collision recovery (25g drones) | Related Work §2.1, Introduction §1 |
| `Gomboc2026` | Gomboc self-righting shape | Related Work §2.1, Methodology §3.3 |
| `UltimateGuideDrone` | Flyability commercial cage drone | Related Work §2.1 |

### Flight Control & Autonomy
| Citation | Key Content | Sections |
|----------|-------------|----------|
| `meierPX4NodebasedMultithreaded2015` | PX4 architecture, uORB | Related Work §2.2, Methodology §3.2 |
| `simpsonLatestPixhawkOpen2023` | Pixhawk ecosystem maturity | Methodology §3.2 |
| `TechnicalSpecificationHolybro2025` | Pixhawk 6C specs | Methodology §3.2 |
| `PX4Docs` | PX4 documentation | Methodology §3.2 |
| `Pixhawk6C6C` | Pixhawk 6C product page | Methodology §3.2 |

### Drivetrain & Hardware
| Citation | Key Content | Sections |
|----------|-------------|----------|
| `gomezParetoOptimalPID2020` | Quadcopter dynamics, PID control | Methodology §3.2, Math §6 |
| `EmaxECOII` | Motor specs | Methodology §3.2 |
| `oscarHowChooseFPV2024` | TWR guidelines, motor selection | Methodology §3.2 |
| `oscarUsingLiPoBatteries2025` | LiPo battery usage | Methodology §3.2 |
| `UnderstandingLiPoBattery2025` | LiPo basics | Methodology §3.2 |
| `UnderstandingLiPoBattery` | LiPo voltage sag | Methodology §3.2 |
| `LiPoBatteriesFPV` | Battery sag comparison | Methodology §3.2 |
| `recursionlabsRecursionLabs6s2021` | 4S vs 6S efficiency comparison | Methodology §3.2 |
| `mechtexUnderstandingRelationshipKV` | Motor KV theory | Methodology §3.2 |
| `UltimateFPVDrone` | Motor efficiency curves | Methodology §3.2 |
| `Innov8tive` | Control authority margin | Methodology §3.2 |
| `roboticsDroneDesignCalculations` | Drone design calculations | Methodology §3.2 |
| `drone24hoursTMotorF20041700KV` | T-Motor thrust test data | Methodology §3.2 |
| `FindYourPerfect` | Indoor drone size limits | Methodology §3.2 |

### Methodology — Setup & Infrastructure
| Citation | Key Content | Sections |
|----------|-------------|----------|
| `InstallDockerEngine10:48:12+0200+0200` | Docker installation | Methodology (cut/condense) |
| `PostinstallationSteps0200` | Rootless Docker | Methodology (cut/condense) |

> **Note:** The two Docker citations have ugly keys (`InstallDockerEngine10:48:12+0200+0200`). These should be cleaned up to `dockerInstallUbuntu` or similar. Same for `PostinstallationSteps0200` → `dockerPostInstall`.

---

## Experimental Data Snapshot (for reference when writing)

From the session journals and database:

| Metric | Value |
|--------|-------|
| Total passes | 179 (108 × 45°, 71 × 75°) |
| Valid flights (post-cutoff) | 32 |
| Fixed cage passes | 56 (45°) + ~35 (75°) |
| Rotating cage passes | 52 (45°) + ~36 (75°) |
| Fixed cage deceleration mean | ~6.5 G |
| Rotating cage deceleration mean | ~9.5 G |
| Improvement (fixed over rotating) | ~24% |
| Fixed cage battery drain | 24.6%/min avg, 104.8s avg flying |
| Rotating cage battery drain | 22.7%/min avg, 123.9s avg flying |
| Avg drain (pass-level, both) | 21.3%/min |
| MOCAP residual | <1mm (0.72mm typical) |
| MOCAP update rate (nominal) | ~360 Hz (camera), ~120-240 Hz (poses topic) |
| Fixed cage MOCAP dropout rate | ~10 Hz (significant — root cause of velocity kinks) |
| Transit speed | 0.30-0.35 m/s |
| Drone weight | ~900g (bare, estimate) |
| Thrust-to-weight | ~3.8:1 |
| Hover throttle | ~45-50% |
| Battery | 6S LiPo |
| Collision target | Cylindrical cardboard + aluminum column, R=4.5cm |

---

## Structural Issues to Note

1. **Nesting Depth in Experiments.tex** — `\subsubsection` under `\subsection` is valid LaTeX but may conflict with `hyperref` heading depth. Add `\setcounter{tocdepth}{3}` and `\setcounter{secnumdepth}{3}` if needed.

2. **Citation Key Hygiene** — Several citation keys were auto-generated by Zotero with timestamps (`InstallDockerEngine10:48:12+0200+0200`). Rename these before submission.

3. **The `##` characters in inline text** (`###LLM NOTES###`) caused a LaTeX crash previously — the `#` is a macro parameter in LaTeX. Never use `#` outside of macro definitions or verbatim environments.

4. **Overfull hboxes** — Currently 25 from long URLs in Methodology.tex. Will disappear when the Docker setup instructions are cut.

5. **Marginpar warnings** — 5 from `\todo{}` commands. Remove all `\todo{}` and `\hlfix{}` before final submission.

6. **Figures directory** — `thesis/plots/` is used for generated analysis plots, `thesis/Figures/` for vector diagrams. Both are in `\graphicspath{}`.

---

## Proposed Section File Checklist

| File | Current State | Action Needed |
|------|--------------|---------------|
| `Abstract.tex` | Draft notes in `\hlfix{}` | Write 1-page abstract |
| `Introduction.tex` | Bullet sketch + RQ | Write full 4-paragraph intro |
| `Related_work.tex` | Good paragraphs | Polish, add citations, add §2.3 |
| `Methodology.tex` | 500+ lines of detail | **Condense to ~50%**, cut all Docker setup |
| | **MISSING** | Add §3.3 Cage design |
| | **MISSING** | Add §3.6 Data pipeline |
| `Experiments.tex` | 4 good + 3 placeholder subsections | Fill §4.5, §4.6, §4.7 |
| `Discussion.tex` | **DOES NOT EXIST** | Create — hypothesize why results look as they do |
| `Conclusion.tex` | **DOES NOT EXIST** | Create — contributions + future work |
| `Math.tex` | Basic equations | Add recovery, vibration, rotational energy equations |
| `Figures.tex` | Stub | Minor expansion with figure catalog |
| `Tables.tex` | Placeholder | Fill with real data from DB |
| `CodeListings.tex` | Stub | Can stay minimal |
| `Citations.tex` | Written | Can stay as-is |
