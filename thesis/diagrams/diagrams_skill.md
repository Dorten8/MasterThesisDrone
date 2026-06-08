# Diagram Styling Rules

This file defines the strict color-coding schema for all system diagrams (e.g., Mermaid, PlantUML, Miro blocks). Any LLM generating or modifying diagrams for this project must strictly adhere to these visual guidelines to ensure architectural consistency.

---

## 1. Node Colors (Components & Directories)

| Component Type | Hex / Color Name | Context / Examples |
| :--- | :--- | :--- |
| **Source Submodules** | `BLUE` | Any submodule located within the `src/` directory. |
| **Flight / Drone Control** | `GREEN` | Anything dealing with flight control or inside the drone control directories. |
| **Hardware** | `GRAY` | Physical components, sensors, actuators, frames. |
| **Operating Systems** | `LIGHT GRAY` | OS layers, firmware environments (e.g., Ubuntu, PX4 NuttX). |
| **Data / Storage** | `ORANGE` | Data blocks, state registries, logs, or databases. |

---

## 2. Edge Colors (Arrows & Connections)

| Link Type | Hex / Color Name | Context / Examples |
| :--- | :--- | :--- |
| **Motion Capture** | `MAGENTA` | Arrows representing MOCAP data tracking, positioning loops, or rigid body telemetry. |
| **Control Signals** | `GREEN` | Actuator commands, setpoints, or guiding inputs. |
| **Data Flow** | `ORANGE` | General topic data, telemetry streams, and data logging pipelines. |

---

## 3. General Architecture Principles

* **Holistic View:** Maintain a clear separation between physical hardware (`GRAY`) and the software abstraction layers (`BLUE`/`GREEN`) interacting with them.
* **Fallback:** Components that do not explicitly fit the categories above should use a neutral, default styling unless otherwise specified, ensuring the highlighted systems retain their visual impact.