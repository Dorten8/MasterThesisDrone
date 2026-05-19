# Session Journal: 2026-04-26 — Disk Space Crisis & Cleanup Strategy

## Where Are We in the Project

We're in Phase 1 validation—attempting to test PX4-Autopilot simulator build locally before deploying the full ROS2-PX4 pipeline to hardware. Today hit a critical blocker: the Pi's 32GB SD card is completely full (0% available). This prevents the PX4 build and any further development until resolved.

## What We Worked On and Why

### 1. Restarted Pi with Clean Slate
**What:** Rebooted the Pi to start fresh after yesterday's Docker/submodule setup.

**Why:** Ensures a clean environment for testing the PX4 simulator build in isolation.

**Outcome:** Clean restart successful; no corruption detected.

### 2. Attempted PX4 Simulator Build
**What:** Ran `bash /home/ws/src/PX4-Autopilot/Tools/setup/ubuntu.sh` followed by `make px4_sitl`.

**Why:** This is the next critical step per yesterday's journal—validate that PX4-Autopilot builds correctly before moving to full ROS2 integration.

**Hurdle:** Build failed immediately with "E: You don't have enough free space in /var/cache/apt/archives/" and "mkdir: cannot create directory '/home/ws/src/PX4-Autopilot/build': No space left on device". The Pi is completely full.

### 3. Disk Space Forensics
**What:** Investigated where 27GB (of 28GB SD card) is being used.

**Initial findings:**
- `/home/ws`: 1.5GB (source code, build artifacts)
- `/vscode`: 2.8GB (VS Code Server SSH installation)
- `/home/dorten` hidden folders: 1.7GB (caches, configs)
- `/usr`: 1.1GB (normal system files)
- Total accounted: ~8.1GB (still 19GB unaccounted for?)

**Clarification needed:** The `du` command showed top-level folders, but there appears to be ~19GB in unaccounted system space. Likely Docker images, system cache, or journal logs.

## Technical Overview of Changes

### Disk Usage Analysis
```
Filesystem      Size  Used Avail Use% Mounted on
overlay          28G   27G     0 100% /

Top space consumers:
- /vscode: 2.8GB (VS Code Server on Pi)
- /home/dorten/.*: 1.7GB (caches/configs)
- /home/ws/src: 1.2GB (source code—KEEP)
- /usr: 1.1G (system)
- /home/ws/build: 182MB (build artifacts—safe to delete)
- /home/ws/install: 68MB (build output—safe to delete)
- /home/ws/log: 39MB (old logs—safe to delete)
```

### Cleanup Strategy Identified
**Safe to delete immediately:**
- `/vscode` (2.8GB) — VS Code Server cache, will rebuild if needed
- `/home/dorten/.cache/` (unknown size, likely 0.5–1GB) — app caches, safe
- `/home/ws/build/`, `/home/ws/install/`, `/home/ws/log/` (~289MB) — rebuild artifacts

**Total recoverable: ~4.5GB**

**Not deleted yet:** User needed to leave before executing cleanup.

## Outcome

**Status:** Session ended with clear root cause identified and cleanup plan established. Build blocked; awaiting space cleanup.

**Deliverables:**
- ✅ Identified disk full blocker (0% available on 28GB SD)
- ✅ Mapped space usage across Pi filesystem
- ✅ Quantified cleanup strategy (~4.5GB recoverable)
- ✅ Determined safe vs. risky deletion targets

**Not Done Yet:**
- ⏳ Execute cleanup (`sudo rm -rf /vscode` and `rm -rf /home/dorten/.cache/`)
- ⏳ Verify disk space recovery with `df -h`
- ⏳ Retry PX4 simulator build (`make px4_sitl`)
- ⏳ Validate build succeeds or fails cleanly

## Learning Summary

**Key Concepts Clarified:**

1. **Disk space in constrained environments (Raspberry Pi):**
   - 32GB SD card is tight for full ROS2 + PX4-Autopilot + Docker ecosystem
   - VS Code Server remote installations can consume several GB
   - Build artifacts (cmake cache, object files) accumulate quickly

2. **Forensic tools for Linux disk usage:**
   - `df -h`: Shows overall filesystem usage and available space
   - `du -sh <path>`: Recursive size of a directory (slow on large trees)
   - `du -sh /* | sort -h`: Find biggest top-level folders
   - `ncdu /`: Interactive disk usage analyzer (not yet installed)
   - GNOME Files List View: Shows file sizes in GUI

3. **Safe cleanup hierarchy on build systems:**
   - **Safest first:** Build artifacts (`build/`, `install/`, `*.o`), caches (`~/.cache/`)
   - **Careful:** System cache (`/var/cache/apt/`), logs (`/var/log/`)
   - **Keep:** Source code, configuration files, version control metadata

4. **Architecture constraint discovered:**
   - Pi's 32GB card may be undersized for this workload (ROS2 + PX4-Autopilot + Docker).
   - Consider moving to larger SD card or external storage if cleanup doesn't provide enough headroom.

## Next Steps

**Immediate (next session):**

1. Execute cleanup on Pi:
   ```bash
   sudo rm -rf /vscode
   rm -rf /home/dorten/.cache/
   rm -rf /home/ws/build /home/ws/install /home/ws/log
   df -h  # Verify space recovered
   ```

2. Retry PX4 simulator build:
   ```bash
   cd /home/ws/src/PX4-Autopilot/
   make px4_sitl
   ```

3. If build succeeds → validate Docker container build (`docker build -f .devcontainer/Dockerfile`)

4. If build still fails → investigate remaining space hogs (likely system journals or Docker images)

**If cleanup + retry still fails:**
- Consider moving PX4-Autopilot to external USB drive
- Or implement shallow clone strategy to reduce source footprint
- Or use the Pi only as a target, build on laptop's devcontainer

**Critical path:** Cleanup → Rebuild → Docker validation → Full ROS2-PX4 pipeline test
