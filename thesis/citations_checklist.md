# Citation Checklist — Master Thesis

**Status:** ✅ In .bib and .tex correctly | 🔴 Key mismatch fixed | ⚠️ Needs Zotero entry

*Last updated: 2026-06-08 — after Zotero sync + .tex fixes*

---

## ✅ All Good — Keys match between .bib and .tex

| Cite Key | File(s) | Notes |
|---|---|---|
| `azambujaWhenBeingSoft2022` | Related_work.tex | CogniFly |
| `lercheSpinFlyUAVSystem` | Related_work.tex, Methodology.tex | SpinFly |
| `mahmudDevelopmentCollisionResilient2021` | Related_work.tex | Collision-resilient drone |
| `mulgaonkarRobustAerialRobot2018` | Related_work.tex | Robust swarms |
| `UltimateGuideDrone` | Related_work.tex | Flyability cage guide |
| `gomezParetoOptimalPID2020` | Methodology.tex | PX4 PID tuning (was `gomezParetoPID2020`) |
| `FindYourPerfect` | Methodology.tex | 4-inch indoor flight limit |
| `Gomboc2026` | Related_work.tex | Gömböc Wikipedia (was `gomboc`) |
| `meierPX4NodebasedMultithreaded2015` | Methodology.tex | PX4 paper (was `Meier2015`) |
| `simpsonLatestPixhawkOpen2023` | Methodology.tex | Pixhawk adoption stat (was `UST2023`) |
| `TechnicalSpecificationHolybro2025` | Methodology.tex | Pixhawk 6C specs (was `Holybro2025`) |
| `Pixhawk6C6C` | Methodology.tex | ArduPilot Pixhawk 6C page (was `ArduPilotDocs`) |
| `oscarHowChooseFPV2024` | Methodology.tex | Oscar Liang motor guide (was `OscarLiang2025`) |
| `oscarUsingLiPoBatteries2025` | Methodology.tex | Oscar Liang LiPo guide (was `OscarLiang2025BegGuide`) |
| `UltimateFPVDrone` | Methodology.tex | LIGPower motor guide (was `LIGPower2025`) |
| `roboticsDroneDesignCalculations` | Methodology.tex | Tyto Robotics (was `TytoRobotics2025`) |
| `EmaxECOII` | Methodology.tex | EMAX motor page (was `EMAX2025`) |
| `mechtexUnderstandingRelationshipKV` | Methodology.tex | KV rating explainer (was `MechTex2024`) |
| `recursionlabsRecursionLabs6s2021` | Methodology.tex | Recursion Labs 6S vs 4S |
| `UnderstandingLiPoBattery2025` | Methodology.tex | Large Battery voltage guide |
| `UnderstandingLiPoBattery` | Methodology.tex | Hanery internal resistance (was `Hanery`) |
| `LiPoBatteriesFPV` | Methodology.tex | China Hobby Line (was `ChinaHobbyLine2025`) |
| `InstallDockerEngine10:48:12+0200+0200` | Methodology.tex | Docker install (was `docker:install`) |
| `SetupROS2` | Methodology.tex | ROS2 Docker guide |
| `PostinstallationSteps0200` | Methodology.tex | Docker post-install (was `docker:postinstall`) ✅ Already fixed earlier |
| `drone24hoursTMotorF20041700KV` | Methodology.tex | Italian T-Motor thrust test site |

---

## ⚠️ Still Missing from `references.bib` — Add to Zotero

These 5 keys are still used in your .tex but NOT in `references.bib`:

### 1. `PX4Docs`
#### exists as: Pixhawk 6C 6C Mini Flight Controller — Copter documentation

### 2. `Innov8tive`
#### deleted use: 
How to Choose FPV Drone Motors - Considerations and Best Motor Recommendations

### 3. `UnderstandingLiPoBattery2025`
#### deleted, use existing citation: Using LiPo Batteries for FPV Drones: Beginner's Guide with Top Product Recommendations

### 4. `LiPoBatteriesFPV`
#### deleted, use existing citation: Using LiPo Batteries for FPV Drones: Beginner's Guide with Top Product Recommendations

### 5. `recursionlabsRecursionLabs6s2021`
#### deleted, use existing citation: Using LiPo Batteries for FPV Drones: Beginner's Guide with Top Product Recommendations

---

## 📋 Section D — Plain-text References (Not \cite'd)

| # | Topic | File | Status |
|---|---|---|---|
| D1 | ArduPilot GPL vs PX4 BSD licensing | Related_work.tex | Needs source |
| D2 | Scaling law — bigger drone = bigger collision penalty | Related_work.tex | `petrisCollisiontolerantAerialRobots2022` may cover this |
| D3 | Your own Z-axis experiment data | Experiments.tex | Just reference your own figures |
| D4 | Raspberry Pi 5 power specs | Methodology.tex | Needs source |

---

## 🗑️ Removed / Consolidated

| Old Key | Reason | Replaced By |
|---|---|---|
| `RCDroneTop2024` | Deleted from thebibliography | `oscarHowChooseFPV2024` (covers TWR tables) |
| `MyFPVStore2025` | Duplicate EMAX page | `EmaxECOII` (same product, official source) |
| `EMAXDatasheet` | Vague source | `drone24hoursTMotorF20041700KV` + `oscarHowChooseFPV2024` |
| `AviationDrones` | No URL, vague | `oscarUsingLiPoBatteries2025` (covers voltage sag) |
| `LIGPower2025` | Key mismatch | `UltimateFPVDrone` |
| `TytoRobotics2025` | Key mismatch | `roboticsDroneDesignCalculations` |
| `EMAX2025` | Key mismatch | `EmaxECOII` |
| `MechTex2024` | Key mismatch | `mechtexUnderstandingRelationshipKV` |
| `LargeBattery2025` | Key mismatch | `UnderstandingLiPoBattery2025` |
| `ChinaHobbyLine2025` | Key mismatch | `LiPoBatteriesFPV` |
| `Hanery` | Key mismatch | `UnderstandingLiPoBattery` |
| `OscarLiang2025` | Key mismatch | `oscarHowChooseFPV2024` |
| `OscarLiang2025BegGuide` | Key mismatch | `oscarUsingLiPoBatteries2025` |
| `RecursionLabs2021` | Key mismatch | `recursionlabsRecursionLabs6s2021` |
| `Meier2015` | Key mismatch | `meierPX4NodebasedMultithreaded2015` |
| `UST2023` | Key mismatch | `simpsonLatestPixhawkOpen2023` |
| `Holybro2025` | Key mismatch | `TechnicalSpecificationHolybro2025` |
| `ArduPilotDocs` | Key mismatch | `Pixhawk6C6C` |
| `gomezParetoPID2020` | Key mismatch | `gomezParetoOptimalPID2020` |

---

## Quick Start

1. **Add the 5 missing entries** (⚠️ section above) to Zotero with the exact keys shown
2. **Sync Zotero** so `references.bib` updates
3. **D1 & D2** — search for licensing pages and the Petris survey paper
4. **D4** — Raspberry Pi 5 specs (raspberrypi.com)
