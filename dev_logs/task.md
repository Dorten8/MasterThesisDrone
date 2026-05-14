# 📝 Master Task List: Drone Bottleneck Fixes

## 🟢 Phase 1: Foundation (Logging & Sync)
- [x] Configure PX4 Logging (`SDLOG_MODE=1` set in QGC)
- [x] Update EKF2 Height Source (`EKF2_HGT_REF=3` set in QGC)
- [x] Disable Optical Flow (`EKF2_OF_CTRL=0` set in QGC)
- [x] Set Custom Noise Mode (`EKF2_EV_NOISE_MD=1` set in QGC)
- [x] Tighten Position Trust (`EKF2_EVP_NOISE=0.05` set in QGC)
- [x] **Synchronize System Clocks**
    - [x] Create [dev_logs/check_clock_sync.sh](file:///home/dorten/pi_drone_sshfs/dev_logs/check_clock_sync.sh)
    - [x] Verify alignment with Mocap PC
    - [x] Run before each session; confirm both Pi and Mocap PC use internet NTP
- [ ] Update `config/fc_pixhawk_6c.params` backup with all new QGC values

## 🟡 Phase 2: Measure & Set `EKF2_EV_DELAY` ✅
- [x] Ground test: hold drone in Mocap area, do sharp manual jerks
- [x] Record bag with `/poses` AND `/fmu/out/sensor_combined` (IMU)
- [x] Find peak-to-peak delta between Mocap spike and IMU spike = true delay
- [x] Set `EKF2_EV_DELAY = 60 ms` in QGC (Measured 60ms May 14)

## 🟢 Phase 3: Bridge Robustness (Pose Hold Watchdog) ✅
- [x] Implemented 50Hz timer in `mocap_px4_bridge` — decoupled from Mocap receive rate
- [x] Republishes last known pose with refreshed timestamp during dropouts < 500ms
- [x] Stops publishing after 500ms dropout (lets EKF2 fail safely, not freeze on stale data)

## 🔵 Phase 4: Validation Flight
- [ ] Fly 1m hover test (`drone_control/hover_benchmark.py`)
- [ ] Confirm EKF2 no longer diverges in bag analysis
- [ ] Compare with Flight 1 (082944) results

## ⚪ Phase 5: Master Thesis Data Pipeline
- [ ] Automate bag recording in startup scripts
- [ ] Create Foxglove JSON layout
- [ ] Develop `dev_logs/scripts/bag_to_csv.py` for Jupyter export

---
### 📂 Key Evidence
- **Climb Anomaly:** EKF2 Z teleported **6.86m in 200ms** (Flight 1) due to incorrect `EKF2_HGT_REF`
- **Latency Target:** Measured **60ms** total pipeline lag via Jerk Test on May 14
- **Fusion Health:** Confirmed **100% binary fusion** once startup sequence finishes
- **Watchdog:** Confirmed bridge masks raw Mocap gaps up to **284ms** successfully
