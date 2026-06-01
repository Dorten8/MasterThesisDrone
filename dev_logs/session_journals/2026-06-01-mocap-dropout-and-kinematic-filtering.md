# June 1, 2026 — MoCap Frame Rate Occlusion & Kinematic Filtering Upgrades

## 🔍 Telemetry Artifact Identification
During today's comparative flight analysis of the `<Rotating Cage>` vs. `<Fixed Cage>` configurations, we identified a significant numerical artifact in the MoCap-derived tangential velocity and acceleration profiles. 

### 1. The Symptom
* In both the velocity and acceleration time-series, we observed sharp, sudden, non-physical spikes and "step-like" jumps (specifically between $5.0\text{s}$ and $9.5\text{s}$ in the Fixed Cage flight and a massive vertical spike at $6.8\text{s}$ in the Rotating Cage flight).
* These spikes do not represent the physical dynamics of the drone, which is a rigid-body mass constrained by inertia.

### 2. The Cause (OptiTrack Packet Occlusions)
* Cross-referencing the velocity plots with the **Mocap Publish Rate (Hz)** subplots revealed a perfect, direct correlation:
  * Whenever the `/poses` stream publish rate collides with or drops below the **$30\text{Hz}$ critical failsafe threshold** (due to network latency over SSHFS or temporary camera marker occlusions), the velocity and acceleration calculations explode.
  * When the publish rate is stable and tightly grouped around the nominal $120\text{Hz}$ target, the curves remain beautifully smooth and physically correct.

### 3. The Mathematics of the Explosion
The velocity magnitude is computed from sequential spatial derivatives:
$$v(t) \approx \frac{\Delta x}{\Delta t}$$
* When packets are dropped, the position updates freeze in time. Upon the next successful packet arrival, the drone appears to "teleport" across a large spatial delta $\Delta x$.
* If the queued messages are delivered in a rapid network burst, the recorded time difference $\Delta t$ between successive message timestamps becomes extremely tiny.
* Dividing a large spatial step $\Delta x$ by a compressed temporal gap $\Delta t$ causes the calculated derivative to explode, producing massive, artificial high-frequency spikes.

---

## 🔮 Planned Mitigation: Uniform Grid Resampling & Spline Interpolation

To preserve both the **raw plots** (which demonstrate sensor real-world noise and EKF2 robust autopilot performance for thesis "extra points") and produce **clean publication-grade kinematic profiles**, we plan to implement a signal processing upgrade:

### The Uniform grid Resampling Algorithm:
1. **Detect Jitter:** Identify samples where the local publish rate drops below $50\text{Hz}$ ($\Delta t > 0.02\text{s}$).
2. **Resample to a Uniform Grid:** Instead of running the Savitzky-Golay filter on non-uniformly sampled timestamps (which mathematically violates the filter's constant-$dt$ assumption), we will resample the position signals ($x, y, z$) onto a perfectly uniform $100\text{Hz}$ grid.
3. **Cubic Spline Interpolation:** Use a cubic spline to interpolate coordinates across any packet-loss gaps.
4. **Savitzky-Golay Derivative Calculation:** Apply the Savitzky-Golay filter to the uniformly resampled position grid. This produces beautifully smooth, physically truthful velocity and acceleration curves that are completely immune to network buffering spikes!
