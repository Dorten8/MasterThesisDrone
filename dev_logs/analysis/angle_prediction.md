# 🎯 Angle Prediction — EDA & Planning

## Context

This document tracks the Exploratory Data Analysis and planning for predicting `impact_angle` from high-frequency IMU data. The EDA found **very strong correlations** — the IMU data clearly fingerprints the impact angle, making regression models viable for the next phase.

## Database: flights_summary

| Filter | Rows |
|---|---|
| Total flights | 179 |
| Fixed Cage, impact_detected=1 | 70 (of 91 Fixed Cage) |
| Rotating Cage, impact_detected=1 | 87 (of 88 Rotating Cage) — *future work* |

## EDA Results (Fixed Cage, N=70)

### Top-10 IMU Features by |Pearson r| with impact_angle

| Rank | Feature | r | p |
|---|---|---|---|
| 1 | Peak Gyro Y | −0.8426 | < 0.001 *** |
| 2 | Peak Accel X | −0.8111 | < 0.001 *** |
| 3 | Vibration Gyro Y | −0.7979 | < 0.001 *** |
| 4 | Accel Spread (Impact) X | −0.7976 | < 0.001 *** |
| 5 | Gyro Energy Y | −0.7874 | < 0.001 *** |
| 6 | Peak Accel Y | −0.7272 | < 0.001 *** |
| 7 | Accel Spread (Impact) Y | −0.7066 | < 0.001 *** |
| 8 | Vibration Gyro X | −0.6934 | < 0.001 *** |
| 9 | Accel Spread (Impact) Z | −0.6920 | < 0.001 *** |
| 10 | Peak Accel Z | −0.6891 | < 0.001 *** |

**All top-10 correlations are NEGATIVE** — steeper (more head-on) impact angles produce lower IMU readings.

### Key Physical Insight

Shallow/grazing impacts (~20°) transfer more **rotational** energy to the drone — the collision cage contacts asymmetrically, inducing pitch/yaw rotation. Head-on impacts (~90°) transfer energy linearly through the cage, reducing rotational disturbance. This explains why **Gyro Y (pitch rate)** is the strongest predictor — the pitch axis captures the rotational asymmetry of oblique impacts.

### Weak / Non-Significant Features

| Feature | r | p |
|---|---|---|
| Accel Settling Time | −0.097 | 0.423 |
| Vibration Accel X | −0.059 | 0.627 |

These can be dropped from any model.

### Battery Confounding Check

The top-3 scatter plots color points by `battery_at_start`. No strong vertical gradient is visible — battery state does not appear to confound the IMU-angle relationship. However, this should be formally checked via partial correlation or by including battery as a control variable in the model.

## Deliverables Created

| File | Description |
|---|---|
| `eda/eda_angle_prediction.py` | Reusable module: `load_impact_data()`, `plot_correlation_heatmap()`, `plot_top3_scatter()`, `plot_parallel_coordinates()` |
| `experiments_angle_prediction.ipynb` | Jupyter notebook that runs the EDA + RF pipeline step by step with commentary |
| `graphics/eda_correlation_heatmap.png` | Single-panel heatmap — 26 IMU features sorted by |r|
| `graphics/eda_top3_scatter.png` | 3-row scatter (Peak Gyro Y, Peak Accel X, Vibration Gyro Y) with Huber trendline |
| `graphics/eda_parallel_coordinates.png` | Parallel coordinates colored by angle bin |

## Next Steps

1. **Full pairwise correlation matrix** among top-10 features — check for multicollinearity before regression
2. **Linear regression baseline** using top-3 features, with Huber loss
3. **Expand to Rotating Cage** — compare coefficient signs/magnitudes
4. **Cross-validation** — leave-one-flight-out (passes from same flight aren't independent)
5. **Explore non-linear models** (Random Forest) if residuals show curvature

## ML-Ready Column Set

Keep for modeling (|r| > 0.5): all top-10 features + `imu_accel_energy_*` (3 cols) = **13 features**.
Drop: `imu_accel_settling`, `imu_vib_ax` (both |r| < 0.1).

**Target**: `impact_angle` (continuous, 20°–88°)
**Control**: `battery_at_start` (if confounding is significant)
