import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline, interp1d

def resample_and_interpolate_mocap(df_mocap, target_freq=100.0, filter_half_window=10):
    """Resamples MoCap coordinates onto a uniform grid using simple linear interpolation.

    Detects dropout gaps and marks the "ringing regions" (gap + SG window margin) so they
    can be surgically removed from the velocity/acceleration arrays later.

    Design rationale for key parameter choices:
    - 100 Hz target frequency: The highest temporal resolution at which all three sensor
      pipelines (MoCap ~120 Hz native, IMU 100 Hz, EKF 100-250 Hz) can be aligned onto a
      single uniform grid without up-sampling interpolated noise. 100 Hz also satisfies
      the Nyquist criterion for drone dynamics (rigid-body modes << 20 Hz), so no
      information is lost.

    - Linear interpolation (not PCHIP/spline): MoCap dropouts in the Fixed Cage produce
      abrupt ~100 ms gaps. PCHIP and cubic splines would "bridge" these gaps with
      physically impossible smooth curves, generating phantom velocity transients when
      differentiated. Linear interpolation across gaps preserves the raw measurement
      character and produces a sharp kink -- which is easier to detect and surgically
      remove in the downstream ringing-removal step than a smoothly interpolated artifact.

    - Ringing mask for SG filter artifact suppression: The Savitzky-Golay filter fits a
      local polynomial over a sliding window. When that window straddles a dropout gap
      (where linear interpolation produced a sharp corner), the polynomial fit overshoots,
      producing a characteristic "ringing" spike in the derivative. Rather than trying to
      prevent this (which would require disabling SG filtering at precisely the right
      moments), we let it happen and then surgically excise the contaminated samples using
      THIS ringing mask. The mask marks the gap region PLUS the SG half-window margin on
      each side -- the full spatial extent of the ringing contamination.
    """
    if df_mocap.empty or len(df_mocap) < 4:
        return df_mocap

    # Ensure unique and monotonically sorted timestamps
    df_clean = df_mocap.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)
    t_start = df_clean['t'].min()
    t_end = df_clean['t'].max()

    # Generate perfectly uniform timeline
    dt = 1.0 / target_freq
    t_uniform = np.arange(t_start, t_end + dt/2.0, dt)

    resampled_data = {'t': t_uniform}
    
    # Simple linear interpolation for all columns (preserves raw data feel, no PCHIP jitter)
    for col in df_clean.columns:
        if col != 't':
            try:
                resampled_data[col] = np.interp(t_uniform, df_clean['t'], df_clean[col])
            except Exception:
                resampled_data[col] = df_clean[col].iloc[0]

    df_resampled = pd.DataFrame(resampled_data)
    
    # ==================================================================
    # Rotating Cage: very clean tracking (~7 drops). Fixed Cage: severe jitter (~146 drops).
    #
    # The two cage configurations produce qualitatively different MoCap
    # tracking quality, requiring different dropout-repair strategies:
    #
    #   Rotating Cage (~7 drops in a 15 s flight):
    #     The cage structure rotates predictably, so retroreflective markers
    #     are rarely occluded. Raw dt rarely exceeds 1/30 s = ~33 ms.
    #     We treat ANY dt > 1/30 s as a genuine dropout and apply the full
    #     SG half-window margin to suppress ringing on both sides of the gap.
    #
    #   Fixed Cage (~146 drops in a 15 s flight):
    #     The static cage beams create persistent marker occlusion --
    #     line-of-sight to 6+ cameras is frequently blocked. The MoCap
    #     solver can still reconstruct position from partial marker sets,
    #     but with dt jitter (many ~50 ms gaps, not true dropouts).
    #     Aggressively flagging every dt > 1/30 s would mark >50% of the
    #     timeline as "ringing" and erase most of the velocity signal.
    #     Strategy: ignore micro-jitter (<100 ms) and ONLY repair massive
    #     structural gaps (>100 ms) that the solver genuinely could not fill.
    #     Use a tiny margin (2 samples) to avoid overlapping repair windows.
    # ==================================================================
    raw_t = df_clean['t'].values
    raw_dt = np.diff(raw_t)

    # A "strict drop" is any inter-sample interval longer than 1/30 s (~33 ms).
    # At 120 Hz native MoCap, the nominal dt is ~8.3 ms. Anything 4x that
    # is almost certainly a tracking loss, not clock jitter.
    
    strict_drops = np.where(raw_dt > (1.0 / 30.0))[0]
    # Heuristic threshold: >20 strict drops in a single flight implies
    # the Fixed Cage jitter regime. Normal Rotating Cage flights have ~7.
    is_high_jitter = len(strict_drops) > 20

    if is_high_jitter:
        # FIXED CAGE PROFILE: Ignore micro-jitter. Only repair massive structural gaps.
        # 100 ms is ~12 missing samples at 120 Hz -- a genuine tracking loss
        # that the solver cannot reconstruct from partial marker data.
        dropout_indices = np.where(raw_dt > 0.1)[0]
        # margin=2: only 2 samples on each side. Larger margins would chain
        # adjacent gaps together and erase too much valid signal.
        margin = 2
    else:
        # ROTATING CAGE PROFILE: Every dt > 1/30 s is a true dropout.
        dropout_indices = strict_drops
        # Full SG half-window margin: the polynomial fit at window boundaries
        # is least constrained, so ringing propagates ~(window//2) samples
        # beyond the gap edge. We excise the full contaminated zone.
        margin = filter_half_window
    
    # Build a boolean mask marking the "ringing regions": the dropout gap
    # itself PLUS the margin on each side where SG polynomial overshoot
    # would contaminate the derivative. Samples where ringing_mask=True
    # will later be replaced by linear interpolation from clean neighbors.
    ringing_mask = np.zeros(len(t_uniform), dtype=bool)
    
    for idx in dropout_indices:
        t0, t1 = raw_t[idx], raw_t[idx + 1]
        gap_indices = np.where((t_uniform >= t0) & (t_uniform <= t1))[0]
        if len(gap_indices) == 0:
            continue
        
        start_idx = max(0, gap_indices[0] - margin)
        end_idx = min(len(t_uniform) - 1, gap_indices[-1] + margin)
        
        ringing_mask[start_idx:end_idx + 1] = True

    df_resampled['_ringing_mask'] = ringing_mask
    df_resampled['_is_high_jitter'] = is_high_jitter

    # Resilient fallback for any floating point boundary NaNs
    df_resampled = df_resampled.ffill().bfill()
    return df_resampled

def compute_mocap_kinematics(df_mocap, window=19, polyorder=3, resample=True, prefilter_position_fc=None):
    r"""Differentiate MoCap positions → velocity → acceleration (Savitzky-Golay pipeline).

    **Standard method: Savitzky-Golay (SG) differentiation.**

    The SG filter fits a polynomial of order *polyorder* over a sliding window
    of *window* samples via linear least squares, then evaluates the analytical
    derivative of that polynomial at the window centre. This is equivalent to
    convolution with pre-computed coefficients — O(N) and numerically stable.

    **Pipeline:**

    1. Resample MoCap positions to a uniform 100 Hz grid via
       `resample_and_interpolate_mocap()`.

    2. SG-differentiate position → velocity:
         v(t) = SG_deriv1( x(t), window=19, polyorder=3 ) / Δt_median      [m/s]

    3. For Fixed Cage (high-jitter): apply 2nd-order Butterworth low-pass,
       fc = 4 Hz, zero-phase (filtfilt) to suppress dropout-kink amplification.

    4. SG-differentiate velocity → acceleration:
         a(t) = SG_deriv1( v(t), window=19, polyorder=3 ) / Δt_median      [m/s²]

    5. Surgically remove ringing artifacts at dropout boundaries by linearly
       interpolating over the ringing mask from step 1.

    **Speed magnitude:**
      s(t) = √( vx(t)² + vy(t)² + vz(t)² )                                 [m/s]

    **Acceleration scalar magnitude:**
      a_s(t) = SG_deriv1( s(t) ) / Δt_median                                [m/s²]

    **Key parameters:**
    - window=19, polyorder=3 (≈190 ms at 100 Hz): Wide enough to smooth mm-level
      MoCap quantisation noise, narrow enough to preserve drone dynamics.
    - Δt_median: Uses median (not mean) dt — robust to remaining outlier gaps.

    **Butterworth post-filter** (Fixed Cage only):
      H(f) = 1 / √(1 + (f / fc)^(2n)),  n=2, fc=4 Hz,  zero-phase via filtfilt.
      Attenuates dropout-frequency noise (~10 Hz) while preserving rigid-body
      drone translation (≪ 4 Hz).

    **Interaction:** Called by `db_pipeline.py` and `flight_loader.py` to produce
    kinematic DataFrames. The velocity/acceleration columns feed into
    `compute_flight_metrics()`. Legacy MoCap path; since 2026-06-10 the default
    is EKF velocity via `compute_ekf_kinematics()`.

    Parameters
    ----------
    prefilter_position_fc : float or None
        If set (e.g. 12.0), applies a 2nd-order Butterworth zero-phase low-pass
        filter to x, y, z positions BEFORE SG differentiation. This band-limits
        the position signal to physically realizable drone dynamics, preventing
        linear interpolation kinks from being amplified into velocity spikes.
        Default None (off) preserves original pipeline behavior.
    """
    if df_mocap.empty or len(df_mocap) < 3:
        df_mocap['vx'] = 0.0
        df_mocap['vy'] = 0.0
        df_mocap['vz'] = 0.0
        df_mocap['speed'] = 0.0
        return df_mocap

    if resample and len(df_mocap) >= 4:
        df_mocap = resample_and_interpolate_mocap(df_mocap, target_freq=100.0, filter_half_window=window//2)

    # Extract adaptive jitter flag
    is_high_jitter = False
    if '_is_high_jitter' in df_mocap.columns:
        is_high_jitter = bool(df_mocap['_is_high_jitter'].iloc[0])
        df_mocap = df_mocap.drop(columns=['_is_high_jitter'])

    # Unique and monotonically sorted
    df_mocap = df_mocap.drop_duplicates(subset=['t']).sort_values('t').reset_index(drop=True)

    # ==================================================================
    # DISABLED: Position Pre-Filtering (2026-06-08)
    # ==================================================================
    # Attempt 1 (FAILED): 12 Hz Butterworth position pre-filter before SG diff.
    # Hypothesis: if we low-pass the position signal at 12 Hz, the linear
    # interpolation kinks (which occur at the dropout gap frequency of ~10 Hz
    # in the Fixed Cage) would be attenuated BEFORE SG differentiation
    # amplifies them. Then we could relax the velocity Butterworth from 4 Hz
    # to 20 Hz, recovering more genuine drone dynamics.
    #
    # Why it failed: The 12 Hz cutoff is too close to the ~10 Hz kink
    # frequency for a 2nd-order filter to provide meaningful attenuation
    # (only ~3 dB at 12 Hz). The kinks still pass through, and relaxing
    # the velocity filter to 20 Hz removes the one component that WAS
    # effectively smoothing the data. Net result: noisier velocity than
    # the original pipeline with the 4 Hz velocity filter in place.
    #
    # Code preserved for future experimentation with steeper filters
    # (e.g., 4th-order or higher) or different cutoff strategies.
    # ==================================================================
    # if prefilter_position_fc is not None:
    #     from scipy.signal import butter, filtfilt
    #     b_pre, a_pre = butter(2, prefilter_position_fc, fs=100.0, btype='low')
    #     for col in ['x', 'y', 'z']:
    #         df_mocap[col] = filtfilt(b_pre, a_pre, df_mocap[col])

    dt = df_mocap['t'].diff()
    median_dt = dt.median()
    if median_dt is None or np.isnan(median_dt) or median_dt == 0:
        median_dt = 0.01

    # Adjust window length to be odd and smaller than len(df_mocap)
    n_points = len(df_mocap)
    adjusted_window = window
    if adjusted_window >= n_points:
        adjusted_window = n_points // 2 * 2 - 1
        if adjusted_window < 3:
            adjusted_window = 3

    # ==================================================================
    # SG DIFFERENTIATION: Position -> Velocity
    # ==================================================================
    # Savitzky-Golay differentiation works by fitting a polynomial of order
    # 'polyorder' over a sliding window of 'window' samples, then evaluating
    # the analytical derivative of that polynomial at the window center.
    # This is equivalent to a convolution with pre-computed coefficients,
    # making it O(N) and numerically stable.
    #
    # deriv=1 gives the first derivative (velocity from position).
    # Division by median_dt converts from [units/sample] to [units/second].
    # Using median (not mean) dt makes the scaling robust to any remaining
    # outlier gaps in the uniform grid.
    #
    # Fallback to np.gradient (2nd-order central differences) when the
    # window is too small for the SG polynomial fit (e.g., very short flights).
    # ==================================================================
    if adjusted_window > polyorder:
        try:
            vx_filtered = savgol_filter(df_mocap['x'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            vy_filtered = savgol_filter(df_mocap['y'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            vz_filtered = savgol_filter(df_mocap['z'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
        except Exception:
            vx_filtered = np.gradient(df_mocap['x'], df_mocap['t'])
            vy_filtered = np.gradient(df_mocap['y'], df_mocap['t'])
            vz_filtered = np.gradient(df_mocap['z'], df_mocap['t'])
    else:
        vx_filtered = np.gradient(df_mocap['x'], df_mocap['t'])
        vy_filtered = np.gradient(df_mocap['y'], df_mocap['t'])
        vz_filtered = np.gradient(df_mocap['z'], df_mocap['t'])

    # ==================================================================
    # BUTTERWORTH LOW-PASS FILTER (Fixed Cage high-jitter only)
    # ==================================================================
    # Applied exclusively in the Fixed Cage regime. Rationale:
    # - SG differentiation of gappy position data amplifies the broad-band
    #   noise introduced by dropout-kink linear interpolation, producing
    #   velocity noise concentrated near the dropout repetition rate (~10 Hz).
    # - A 2nd-order Butterworth with 4 Hz cutoff attenuates this noise band
    #   while preserving rigid-body drone translation dynamics (which are
    #   well below 4 Hz for a 1.5 kg quadcopter at moderate speeds).
    # - filtfilt applies the filter forward and backward, yielding zero
    #   phase distortion. This is critical: any group delay would shift the
    #   timing of the velocity minimum at impact, corrupting the Column
    #   Impact detection downstream.
    # - 2nd order = 12 dB/octave rolloff: gentle enough to avoid ringing
    #   at the cutoff, steep enough to suppress the ~10 Hz noise.
    # ==================================================================
    if is_high_jitter:
        from scipy.signal import butter, filtfilt
        try:
            # 2nd order Butterworth, 4 Hz cutoff, 100 Hz sampling
            b, a = butter(2, 4.0, fs=100.0, btype='low')
            vx_filtered = filtfilt(b, a, vx_filtered)
            vy_filtered = filtfilt(b, a, vy_filtered)
            vz_filtered = filtfilt(b, a, vz_filtered)
        except Exception as e:
            print(f"[Warning] Failed to apply Butterworth low-pass filter: {e}")

    df_mocap['is_high_jitter'] = is_high_jitter
    df_mocap['_prefilter_fc'] = prefilter_position_fc if prefilter_position_fc is not None else 0.0
    df_mocap['vx'] = vx_filtered
    df_mocap['vy'] = vy_filtered
    df_mocap['vz'] = vz_filtered
    df_mocap['speed'] = np.sqrt(vx_filtered**2 + vy_filtered**2 + vz_filtered**2)
    
    # ==================================================================
    # SG DIFFERENTIATION: Velocity -> Acceleration
    # ==================================================================
    # Same SG parameters (window, polyorder, deriv=1) applied to the
    # already-filtered velocity signal. The cascade of SG(position->vel)
    # then SG(vel->accel) is equivalent to a single SG(position->accel)
    # with deriv=2, but applying it in two stages lets us insert the
    # Butterworth filter and ringing removal between stages.
    # ==================================================================
    if adjusted_window > polyorder:
        try:
            df_mocap['ax'] = savgol_filter(df_mocap['vx'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            df_mocap['ay'] = savgol_filter(df_mocap['vy'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            df_mocap['az'] = savgol_filter(df_mocap['vz'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
            df_mocap['accel'] = savgol_filter(df_mocap['speed'], window_length=adjusted_window, polyorder=polyorder, deriv=1) / median_dt
        except Exception:
            df_mocap['ax'] = np.gradient(df_mocap['vx'], df_mocap['t'])
            df_mocap['ay'] = np.gradient(df_mocap['vy'], df_mocap['t'])
            df_mocap['az'] = np.gradient(df_mocap['vz'], df_mocap['t'])
            df_mocap['accel'] = np.gradient(df_mocap['speed'], df_mocap['t'])
    else:
        df_mocap['ax'] = np.gradient(df_mocap['vx'], df_mocap['t'])
        df_mocap['ay'] = np.gradient(df_mocap['vy'], df_mocap['t'])
        df_mocap['az'] = np.gradient(df_mocap['vz'], df_mocap['t'])
        df_mocap['accel'] = np.gradient(df_mocap['speed'], df_mocap['t'])

    # ==================================================================
    # SURGICAL RINGING REMOVAL
    # ==================================================================
    # The ringing_mask (built in resample_and_interpolate_mocap) marks all
    # samples contaminated by SG polynomial overshoot near dropout gaps.
    # Here we replace those contaminated velocity/acceleration samples with
    # a simple linear interpolation between the nearest clean neighbors.
    #
    # Why linear interpolation (not higher-order) for the repair:
    # The contaminated segments are short (a few samples + margin) and the
    # drone velocity changes slowly relative to 100 Hz. A straight line is
    # the least-assumptive repair -- it introduces no phantom dynamics.
    # Higher-order interpolation (cubic, spline) would invent spurious
    # accelerations in the repaired gap, exactly the problem we are trying
    # to solve.
    # ==================================================================
    if '_ringing_mask' in df_mocap.columns:
        is_valid = ~df_mocap['_ringing_mask'].values
        valid_t = df_mocap['t'].values[is_valid]
        
        if len(valid_t) > 0:
            for col in ['vx', 'vy', 'vz', 'speed', 'ax', 'ay', 'az', 'accel']:
                valid_vals = df_mocap[col].values[is_valid]
                df_mocap[col] = np.interp(df_mocap['t'].values, valid_t, valid_vals)
                
        df_mocap['is_doctored'] = df_mocap['_ringing_mask']
        df_mocap = df_mocap.drop(columns=['_ringing_mask'])
    else:
        df_mocap['is_doctored'] = False
            
    return df_mocap

def compute_ekf_kinematics(df_odom, df_mocap, target_freq=100.0, window=19, polyorder=3):
    """Computes velocity, speed, and acceleration from PX4 EKF odometry.

    The PX4 Extended Kalman Filter (EKF) runs on the flight controller and fuses
    two sensor streams into a single state estimate:
      - MoCap position (external vision, ~120 Hz native, subject to dropouts)
      - IMU acceleration + gyro (onboard, 100 Hz, always available)

    The EKF exploits the complementary nature of these sensors: IMU provides
    high-bandwidth acceleration that can be integrated forward during MoCap
    dropouts, while MoCap provides absolute position fixes that prevent IMU
    integration drift. The result is a velocity estimate that is smooth even
    through Fixed Cage occlusions -- the core advantage over the pure MoCap
    pipeline in compute_mocap_kinematics().

    This function:
    1. Applies NED->ENU coordinate alignment (same convention as db_loader.py).
       PX4 uses NED (x=North, y=East, z=Down). Our ENU convention is
       x=East, y=North, z=Up. For velocity: vx stays, vy flips sign, vz flips sign.
    2. Resamples EKF velocity onto a uniform 100 Hz grid matching the MoCap pipeline,
       enabling direct column-wise replacement in compute_flight_metrics().
    3. Differentiates EKF velocity to acceleration using the same SG parameters
       (window=19, polyorder=3, deriv=1) as compute_mocap_kinematics() for consistency.

    Parameters
    ----------
    df_odom : pd.DataFrame
        EKF odometry with columns 't', 'vx_ekf_raw', 'vy_ekf_raw', 'vz_ekf_raw'.
        Generated by build_dataframes() in db_loader.py.
    df_mocap : pd.DataFrame
        MoCap DataFrame (used for time baseline / t_start, t_end).
    target_freq : float
        Target resample frequency (default 100 Hz, matching MoCap pipeline).
    window : int
        Savitzky-Golay window length for acceleration differentiation.
    polyorder : int
        Savitzky-Golay polynomial order.

    Returns
    -------
    pd.DataFrame or None
        DataFrame with columns 't', 'vx', 'vy', 'vz', 'speed', 'ax', 'ay', 'az', 'accel'
        on the resampled time grid. Returns None if df_odom is empty or missing velocity columns.
    """
    if df_odom.empty or 'vx_ekf_raw' not in df_odom.columns:
        return None

    # ==================================================================
    # NED -> ENU COORDINATE ALIGNMENT
    # ==================================================================
    # PX4 publishes odometry in NED frame (x=North, y=East, z=Down).
    # Our analysis uses ENU (x=East, y=North, z=Up), consistent with
    # the OptiTrack MoCap coordinate system.
    #
    #   NED -> ENU:  x_enu =  y_ned    (East)
    #                y_enu =  x_ned    (North)
    #                z_enu = -z_ned    (Up)
    #
    # For velocity, this means: vx stays (East component unchanged),
    # vy flips sign (NED y=East -> ENU y=North, sign convention difference),
    # vz flips sign (NED Down positive -> ENU Up positive).
    # Same convention as build_dataframes() in db_loader.py:
    #   x_ekf = position[0], y_ekf = -position[1], z_ekf = -position[2]
    #   vx_ekf = v[0],      vy_ekf = -v[1],       vz_ekf = -v[2]
    # ==================================================================
    df_vel = df_odom[['t', 'vx_ekf_raw', 'vy_ekf_raw', 'vz_ekf_raw']].copy()
    df_vel = df_vel.dropna().sort_values('t').reset_index(drop=True)
    df_vel['vx'] = df_vel['vx_ekf_raw']
    df_vel['vy'] = -df_vel['vy_ekf_raw']
    df_vel['vz'] = -df_vel['vz_ekf_raw']

    # ==================================================================
    # RESAMPLE EKF VELOCITY TO UNIFORM 100 Hz GRID
    # ==================================================================
    # The EKF publishes odometry at a variable rate (100-250 Hz depending
    # on sensor fusion health). We resample onto the same 100 Hz uniform
    # grid used by the MoCap pipeline so that EKF and MoCap data share
    # identical time coordinates -- this enables direct column-wise
    # replacement in compute_flight_metrics().
    # ==================================================================
    t_start = df_vel['t'].min()
    t_end = df_vel['t'].max()

    # Clip to MoCap time bounds to avoid extrapolation beyond the region
    # where both sensors have valid data. This also prevents EKF data from
    # being used for pre-takeoff or post-landing epochs where the MoCap
    # pipeline has no position reference.
    if not df_mocap.empty:
        t_start = max(t_start, df_mocap['t'].min())
        t_end = min(t_end, df_mocap['t'].max())

    if t_end <= t_start:
        return None

    dt = 1.0 / target_freq
    t_uniform = np.arange(t_start, t_end + dt / 2.0, dt)

    if len(t_uniform) < 3:
        return None

    # Linear interpolation of EKF velocity onto the uniform grid.
    # EKF velocity is already smooth (IMU-fused), so linear interpolation
    # is sufficient -- no ringing risk from the source data, unlike the
    # MoCap position pipeline where dropout gaps require careful handling.
    ekf_t = df_vel['t'].values
    vx_uniform = interp1d(ekf_t, df_vel['vx'].values, kind='linear',
                          bounds_error=False, fill_value='extrapolate')(t_uniform)
    vy_uniform = interp1d(ekf_t, df_vel['vy'].values, kind='linear',
                          bounds_error=False, fill_value='extrapolate')(t_uniform)
    vz_uniform = interp1d(ekf_t, df_vel['vz'].values, kind='linear',
                          bounds_error=False, fill_value='extrapolate')(t_uniform)

    speed_uniform = np.sqrt(vx_uniform**2 + vy_uniform**2 + vz_uniform**2)

    # ==================================================================
    # ACCELERATION: SG DERIVATIVE OF EKF VELOCITY
    # ==================================================================
    # The EKF velocity is already smooth, so SG differentiation produces
    # clean acceleration without needing the Butterworth filter or ringing
    # removal steps required by the MoCap pipeline. We use the same SG
    # parameters (window=19, polyorder=3, deriv=1) for methodological
    # consistency with compute_mocap_kinematics().
    # ==================================================================
    median_dt = np.median(np.diff(t_uniform))
    if median_dt is None or np.isnan(median_dt) or median_dt == 0:
        median_dt = dt

    n_points = len(t_uniform)
    adj_window = window
    if adj_window >= n_points:
        adj_window = n_points // 2 * 2 - 1
        if adj_window < 3:
            adj_window = 3

    if adj_window > polyorder:
        try:
            ax = savgol_filter(vx_uniform, window_length=adj_window, polyorder=polyorder, deriv=1) / median_dt
            ay = savgol_filter(vy_uniform, window_length=adj_window, polyorder=polyorder, deriv=1) / median_dt
            az = savgol_filter(vz_uniform, window_length=adj_window, polyorder=polyorder, deriv=1) / median_dt
            accel = savgol_filter(speed_uniform, window_length=adj_window, polyorder=polyorder, deriv=1) / median_dt
        except Exception:
            ax = np.gradient(vx_uniform, t_uniform)
            ay = np.gradient(vy_uniform, t_uniform)
            az = np.gradient(vz_uniform, t_uniform)
            accel = np.gradient(speed_uniform, t_uniform)
    else:
        ax = np.gradient(vx_uniform, t_uniform)
        ay = np.gradient(vy_uniform, t_uniform)
        az = np.gradient(vz_uniform, t_uniform)
        accel = np.gradient(speed_uniform, t_uniform)

    df_ekf = pd.DataFrame({
        't': t_uniform,
        'vx': vx_uniform,
        'vy': vy_uniform,
        'vz': vz_uniform,
        'speed': speed_uniform,
        'ax': ax,
        'ay': ay,
        'az': az,
        'accel': accel,
    })
    return df_ekf


def detect_mission_class(df_setpoint, label=None, target_z=0.5):
    """Dynamically determines and instantiates the correct mission class based on label or setpoint coordinates."""
    is_75deg = False
    is_45deg = False
    if label:
        lbl_lower = label.lower()
        if "45" in lbl_lower:
            is_45deg = True
        elif "75" in lbl_lower or "collision" in lbl_lower:
            is_75deg = True

    import sys
    import os
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
    if project_root not in sys.path:
        sys.path.append(project_root)

    if is_45deg:
        try:
            from drone_control.missions.exp_collision_45deg import ExpCollision45Deg
            return ExpCollision45Deg(target_z=target_z)
        except Exception:
            pass

    if not is_75deg and not is_45deg:
        try:
            from drone_control.missions.column_sweep_loop import ColumnSweepLoop
            return ColumnSweepLoop(target_z=target_z)
        except Exception:
            return None

    # Check if we should use ExpCollision75DegV2
    use_v2 = False
    if label and "v2" in label.lower():
        use_v2 = True
    elif df_setpoint is not None and not df_setpoint.empty and 'y_cmd' in df_setpoint.columns:
        # Check if Y=1.10 is commanded close to the sweep lane X=0.186
        sp_v2_mask = (df_setpoint['x_cmd'] - 0.186).abs() < 0.05
        if sp_v2_mask.any():
            y_cmds = df_setpoint.loc[sp_v2_mask, 'y_cmd']
            if ((y_cmds - 1.100).abs() < 0.05).any():
                use_v2 = True

    if use_v2:
        try:
            from drone_control.missions.exp_collision_75deg_v2 import ExpCollision75DegV2
            return ExpCollision75DegV2(target_z=target_z)
        except Exception:
            pass
    try:
        from drone_control.missions.exp_collision_75deg import ExpCollision75Deg
        return ExpCollision75Deg(target_z=target_z)
    except Exception:
        return None


def find_waypoint_events(df_mocap, df_setpoint, takeoff_time=None, label=None, column_x=0.408, column_y=0.358,
                         column_radius=0.045, cage_radius=0.179, return_all=False, dynamic_waypoints=None):
    """Detects collision-pass waypoint events using a 3-tier detection strategy.

    This is the central temporal-event detector for the collision experiment
    analysis pipeline. It identifies when the drone reaches each waypoint,
    when it passes the column center, and when the actual impact occurs.

    Detection strategy (3 tiers, in priority order):
    ==================================================

    PRIORITY 1 -- Mission class import (SSoT):
      Imports the Python mission class that was actually executed on the
      flight controller (e.g., ExpCollision75Deg). Calling mission.on_start()
      populates the internal waypoint coordinates (wp_stage, exp_sp, exp_ep,
      wp3) with the exact commanded values. This is the Single Source of Truth
      because it is the same code that ran on the drone.

    PRIORITY 2 -- Dynamic waypoints from MCAP:
      Falls back to waypoints extracted from the ROS bag setpoint topic.
      Only used if EXACTLY 4 waypoints are found (>4 means PX4 trajectory
      interpolation noise, not real waypoints).

    PRIORITY 3 -- Hardcoded fallback:
      Last resort. Uses known coordinates for 45-degree, 75-degree, and
      column-sweep-loop missions. These values are the nominal design
      coordinates from the experiment specification.

    Waypoint-to-event mapping (collision missions):
      WP1 = staging point (wp_stage) -- where the drone waits before sweep
      WP2 = experiment start point (exp_sp) -- sweep entry
      WP3 = experiment end point (exp_ep) -- sweep exit
      WP4 = return point (wp3) -- post-sweep loiter

    After waypoint detection, post-processing refines:
      - WP2 refinement: from command-transition time to actual sweep-start
        (Y=0.70m crossing + last speed below 0.10 m/s)
      - Column Passed: time of minimum |Y - column_y|
      - Column Impact: speed peak within [-0.25, +0.1]s of closest approach
        (identifies onset of deceleration from collision)
    """
    if df_mocap.empty:
        return [] if return_all else {}

    # Backward compatibility handler for legacy signatures:
    # Some older scripts call find_waypoint_events(df_mocap, takeoff_time, label=...)
    # where df_setpoint is omitted entirely.
    if not isinstance(df_setpoint, pd.DataFrame):
        # The second argument is actually takeoff_time!
        takeoff_time = df_setpoint
        df_setpoint = pd.DataFrame()

    # ==================================================================
    # 3-TIER WAYPOINT SEQUENCE DETERMINATION
    # ==================================================================
    # The mission file defines the exact 4 nominal waypoints. Dynamic
    # waypoints extracted from segmented pass MCAPs are unreliable because
    # PX4 trajectory interpolation produces 15+ intermediate setpoints
    # between the real 4 WPs -- we cannot distinguish real waypoints from
    # interpolated waypoints in those streams.
    # ==================================================================
    wp_sequence = None

    # ---- PRIORITY 1: Mission class import (Single Source of Truth) ----
    # Instantiates the actual Python mission class and simulates on_start()
    # to extract the commanded waypoint coordinates. This is the SSoT
    # because the same mission code ran on the flight controller.
    mission = detect_mission_class(df_setpoint, label, target_z=0.5)
    if mission is not None:
        try:
            # Simulate takeoff to populate the mission waypoints
            takeoff_samples = df_mocap[df_mocap['t'] >= takeoff_time]
            if not takeoff_samples.empty:
                x0 = takeoff_samples['x'].iloc[0]
                y0 = takeoff_samples['y'].iloc[0]
                z0 = takeoff_samples['z'].iloc[0]
            else:
                x0, y0, z0 = 0.0, 0.0, 0.5

            class DummyPose:
                def __init__(self, x, y, z):
                    self.x = x
                    self.y = y
                    self.z = z
            
            mission.on_start(DummyPose(x0, y0, z0))
            
            # Check if this is a collision loop or regular loop
            is_collision = False
            if label:
                lbl_lower = label.lower()
                if "collision" in lbl_lower or "75" in lbl_lower or "45" in lbl_lower:
                    is_collision = True

            if is_collision:
                wp_sequence = [
                    ('WP1', (mission.wp_stage[0], mission.wp_stage[1])),
                    ('WP2', (mission.exp_sp[0], mission.exp_sp[1])),
                    ('WP3', (mission.exp_ep[0], mission.exp_ep[1])),
                    ('WP4', (mission.wp3[0], mission.wp3[1]))
                ]
            else:
                wp_sequence = [
                    ('WP1', (mission.wp1[0], mission.wp1[1])),
                    ('WP2', (mission.wp2[0], mission.wp2[1])),
                    ('WP3', (mission.wp3[0], mission.wp3[1])),
                    ('WP4', (mission.wp4[0], mission.wp4[1]))
                ]
        except Exception:
            pass

    # PRIORITY 2: Fall back to dynamic_waypoints ONLY if exactly 4 were found
    # (>4 means PX4 trajectory interpolation noise, not real waypoints)
    if wp_sequence is None and dynamic_waypoints and len(dynamic_waypoints) == 4:
        wp_sequence = []
        for i, (_, coord) in enumerate(dynamic_waypoints):
            wp_sequence.append((f"WP{i+1}", coord))

    # Absolute hardcoded fallback
    if wp_sequence is None:
        if is_collision:
            # Detect which angle from label
            is_45 = label and "45" in label.lower()
            x_lane = 0.248 if is_45 else 0.186
            wp_sequence = [
                ('WP1', (x_lane, 1.200)),
                ('WP2', (x_lane, 0.950)),
                ('WP3', (x_lane, -1.200)),
                ('WP4', (0.000, 0.300))
            ]
        else:
            wp_sequence = [
                ('WP1', (0.000, 1.200)),
                ('WP2', (0.100, 1.200)),
                ('WP3', (0.100, -1.200)),
                ('WP4', (0.000, -1.200))
            ]

    # ==================================================================
    # SEQUENTIAL COMMAND-TRANSITION SEARCH
    # ==================================================================
    # The Flight Director publishes the ACTIVE position command to the
    # /setpoint topic. When the drone reaches WP1 (proximity check passes),
    # the Flight Director transitions the active command to WP2 coordinates.
    # We detect this transition by scanning the setpoint stream for the
    # exact moment the commanded (x, y) switches to the next WP in sequence.
    #
    # This is more robust than MoCap-proximity detection because:
    # - It uses the flight controller's own "waypoint reached" logic.
    # - It works even when MoCap has dropouts near the waypoint.
    # - It automatically handles multi-loop flights by advancing a search
    #   pointer through the timeline.
    #
    # The 5 cm tolerance accounts for floating-point precision in the
    # commanded coordinates (they are published exactly from the mission
    # file constants, but float comparison needs a small epsilon).
    # ==================================================================
    if not df_setpoint.empty and 'x_cmd' in df_setpoint.columns and 'y_cmd' in df_setpoint.columns:
        df_sp = df_setpoint.dropna(subset=['x_cmd', 'y_cmd'])
    else:
        df_sp = pd.DataFrame(columns=['t', 'x_cmd', 'y_cmd'])
    
    # Find ALL loop passes sequentially
    passes_events = []
    current_search_time = takeoff_time
    max_time = df_sp['t'].max() if not df_sp.empty else df_mocap['t'].max()

    while current_search_time < max_time and not df_sp.empty:
        wp_events = {}
        t_ptr = current_search_time
        
        # Sequentially search for transitions between commands in the loop sequence
        found_all = True
        for i, (wp_name, (wx, wy)) in enumerate(wp_sequence):
            # To find when the drone reached `wp_name` (e.g. WP1), we look for when the Flight Director 
            # transitioned to commanding the NEXT waypoint in the sequence!
            if i + 1 < len(wp_sequence):
                next_wp_name, (next_wx, next_wy) = wp_sequence[i + 1]
                # Look for the exact timestamp the command changes to (next_wx, next_wy)
                dist_to_next = np.sqrt((df_sp['x_cmd'] - next_wx)**2 + (df_sp['y_cmd'] - next_wy)**2)
                
                # Match when the commanded setpoint gets within a 5cm mathematical tolerance 
                # (commands are published exactly, but float precision requires small tolerance)
                match_mask = (dist_to_next < 0.05) & (df_sp['t'] > t_ptr)
                if match_mask.any():
                    arrival_t = df_sp.loc[match_mask, 't'].iloc[0]
                    wp_events[wp_name] = arrival_t
                    # Advance search pointer slightly to prevent duplicate matches on same transition
                    t_ptr = arrival_t + 0.1
                else:
                    found_all = False
                    break
            else:
                # For the absolute final waypoint (e.g. WP4), there is no 'next' command in the sequence.
                # Since we only use WP2 -> WP3 for the sweep, we just assign it a dummy completion time.
                wp_events[wp_name] = t_ptr + 2.0
                break
        
        if found_all:
            passes_events.append(wp_events)
            # Advance search pointer past the end of the loop
            current_search_time = wp_events[wp_sequence[-1][0]] + 1.0
        else:
            # If a pass aborted mid-loop, advance search pointer to the NEXT time WP1 is commanded!
            if len(wp_sequence) >= 2:
                # The start of a new loop happens when WP1 (stage) is reached, which means 
                # the command transitions to WP2 (exp_sp).
                next_wx, next_wy = wp_sequence[1][1]
                dist_to_next = np.sqrt((df_sp['x_cmd'] - next_wx)**2 + (df_sp['y_cmd'] - next_wy)**2)
                # Look at least 5.0s ahead to skip the current failed attempt
                next_loop_mask = (dist_to_next < 0.05) & (df_sp['t'] > current_search_time + 5.0)
                if next_loop_mask.any():
                    current_search_time = df_sp.loc[next_loop_mask, 't'].iloc[0] - 0.5
                else:
                    break
            else:
                break

    # ==================================================================
    # FALLBACK: MoCap Proximity Detection
    # ==================================================================
    # Used when no setpoint transitions were found (truncated passes,
    # segmented MCAP exports, or legacy bags without setpoint topics).
    #
    # Algorithm: find the MoCap sample closest to each nominal waypoint
    # coordinate. This is less precise than the command-transition method
    # (it depends on MoCap quality near waypoints) but works universally.
    #
    # WP1 and WP4 times are synthesized as -1.0s and +1.0s offsets from
    # WP2 and WP3 respectively, since only the WP2->WP3 segment is used
    # for metric computation.
    # ==================================================================
    if not passes_events and not df_mocap.empty and wp_sequence is not None:
        try:
            # We search for the timestamps where the drone is closest to the nominal WP2 (entry) and WP3 (exit)
            wp2_coords = wp_sequence[1][1]  # WP2 (exp_sp)
            wp3_coords = wp_sequence[2][1]  # WP3 (exp_ep)
            
            dist_to_wp2 = np.sqrt((df_mocap['x'] - wp2_coords[0])**2 + (df_mocap['y'] - wp2_coords[1])**2)
            dist_to_wp3 = np.sqrt((df_mocap['x'] - wp3_coords[0])**2 + (df_mocap['y'] - wp3_coords[1])**2)
            
            t_wp2 = df_mocap.loc[dist_to_wp2.idxmin(), 't']
            t_wp3 = df_mocap.loc[dist_to_wp3.idxmin(), 't']
            
            # Ensure chronological order
            if t_wp2 < t_wp3:
                wp_evs = {
                    'WP1': t_wp2 - 1.0,
                    'WP2': t_wp2,
                    'WP3': t_wp3,
                    'WP4': t_wp3 + 1.0
                }
                passes_events.append(wp_evs)
        except Exception as e:
            print(f"   ⚠️  Fallback pass detection failed: {e}")

    # ==================================================================
    # POST-PROCESSING: WP2 Refinement + Column Passed + Column Impact
    # ==================================================================
    # The command-transition WP2 time reflects when the Flight Director
    # SWITCHED the commanded setpoint -- not when the drone actually started
    # moving. We refine WP2 to the true sweep-start using two criteria:
    #   1. The drone crosses Y = 0.70m (descending toward the column at
    #      Y = 0.358m). This establishes a "not started yet" upper bound.
    #   2. Within the pre-cross window, find the LAST time speed drops
    #      below 0.10 m/s. This catches the stationary-to-moving transition
    #      at the staging point (the drone pauses at WP1 before sweeping).
    #
    # Column Impact detection:
    #   1. Geometric closest approach to the column center (min Euclidean
    #      distance). This is the "Column Passed" event.
    #   2. If min distance <= 0.38m (column_radius + cage_radius + margin),
    #      search for a speed PEAK in [t_geom - 0.25, t_geom + 0.1]s.
    #      The speed peak identifies the onset of deceleration -- the
    #      moment just before the collision impulse slows the drone.
    # ==================================================================
    for wp_evs in passes_events:
        if 'WP2' in wp_evs and 'WP3' in wp_evs:
            t_wp2_cmd = wp_evs['WP2']
            wp_evs['WP2_cmd'] = wp_evs['WP2']  # preserve command time before refinement overwrites WP2
            t_wp3 = wp_evs['WP3']
            
            # Refine WP2 to be the actual start of sweep (when it locks and goes) rather than the command transition time.
            df_window = df_mocap[(df_mocap['t'] >= t_wp2_cmd) & (df_mocap['t'] <= t_wp3)]
            if not df_window.empty:
                # Find when the drone crosses Y = 0.70m towards WP3 (Y is decreasing towards WP3 = -1.2)
                cross_mask = df_window['y'] <= 0.70
                t_cross = df_window.loc[cross_mask, 't'].iloc[0] if cross_mask.any() else t_wp3
                
                df_pre_cross = df_window[df_window['t'] <= t_cross]
                if not df_pre_cross.empty:
                    # Find the last time speed is below 0.10 m/s (ignoring NaNs)
                    valid_speeds = df_pre_cross['speed'].dropna()
                    low_speed_mask = valid_speeds < 0.10
                    
                    if low_speed_mask.any():
                        t_start_sweep = df_pre_cross.loc[valid_speeds[low_speed_mask].index[-1], 't']
                    elif not valid_speeds.empty:
                        # Fallback to minimum speed in pre-cross window
                        idx_min = valid_speeds.idxmin()
                        t_start_sweep = df_pre_cross.loc[idx_min, 't']
                    else:
                        # Fallback if entire window is NaN-masked
                        t_start_sweep = df_pre_cross['t'].iloc[-1]
                    
                    # Update WP2 to the refined timestamp
                    wp_evs['WP2'] = t_start_sweep
            
            t_wp2 = wp_evs['WP2']
            sweep_data = df_mocap[(df_mocap['t'] >= t_wp2) & (df_mocap['t'] <= t_wp3)]
            if not sweep_data.empty:
                idx_min = (sweep_data['y'] - column_y).abs().idxmin()
                wp_evs['Column Passed'] = sweep_data.loc[idx_min, 't']
                
                # Detect exact Column Impact Event inside this specific pass (closest approach)
                dist_profile = np.sqrt((sweep_data['x'] - column_x)**2 + (sweep_data['y'] - column_y)**2)
                if not dist_profile.empty:
                    idx_min_dist = dist_profile.idxmin()
                    min_dist = dist_profile.loc[idx_min_dist]
                    
                    # Proximity check: ensure the pass is within a close proximity envelope of 0.38m
                    if min_dist <= 0.38:
                        t_geom = sweep_data.loc[idx_min_dist, 't']
                        
                        # Onset of deceleration (speed peak search around closest approach t_geom)
                        window_mask = (sweep_data['t'] >= t_geom - 0.25) & (sweep_data['t'] <= t_geom + 0.1)
                        window_data = sweep_data[window_mask]
                        if not window_data.empty:
                            idx_peak_speed = window_data['speed'].idxmax()
                            wp_evs['Column Impact'] = window_data.loc[idx_peak_speed, 't']
                        else:
                            wp_evs['Column Impact'] = t_geom

    if return_all:
        return passes_events
    else:
        return passes_events[0] if passes_events else {}

def query_battery(df_bat, t_query):
    """Helper to query battery remaining percentage and voltage at a specific timestamp.

    Uses np.searchsorted for O(log N) binary search on the sorted battery
    time index, then clamps to valid bounds.
    """
    if df_bat is None or df_bat.empty:
        return "N/A", "N/A"
    # Binary search for the insertion point of t_query in the sorted
    # battery timestamp array. Returns the index of the nearest sample.
    idx = np.searchsorted(df_bat['t'], t_query)
    idx = min(max(0, idx), len(df_bat) - 1)
    return f"{df_bat['remaining'].iloc[idx]:.1f}%", f"{df_bat['voltage'].iloc[idx]:.2f}V"

def build_events_log(df_mocap, df_bat, arming_time, takeoff_time, disarming_time, wp_events, achieved_angle=None):
    """Compiles a chronological flight events table for display.

    Assembles a human-readable event timeline by walking through the known
    temporal landmarks (arming, takeoff, waypoint arrivals, column impact,
    disarming) in chronological order. For each event, the inner add_event()
    closure records:
      - Absolute and relative (to arming) timestamps
      - MoCap position (ENU coordinates) at that instant
      - Segment-average speed from the previous event to this one
      - Battery remaining percentage and voltage

    Waypoint events are sorted by their timestamps before insertion to
    maintain chronological order regardless of wp_events dict order.

    The segment-average velocity is the arithmetic mean of the 'speed'
    column over [last_t_abs, t_abs], giving a representative cruise speed
    for the flight segment between two consecutive events.
    """
    events_log = []
    # last_t_abs tracks the previous event's timestamp for computing
    # segment-average velocity between consecutive events.
    last_t_abs = None

    def add_event(name, t_abs):
        """Inner closure: queries position, battery, and segment velocity,
        then appends a formatted event dict to events_log."""
        nonlocal last_t_abs
        if t_abs is None or np.isnan(t_abs):
            return
        t_arm_rel = t_abs - arming_time
        bat_pct, bat_v = query_battery(df_bat, t_abs)
        
        # Query position
        if not df_mocap.empty:
            idx = np.searchsorted(df_mocap['t'], t_abs)
            idx = min(max(0, idx), len(df_mocap) - 1)
            pos_str = f"({df_mocap['x'].iloc[idx]:.3f}, {df_mocap['y'].iloc[idx]:.3f}, {df_mocap['z'].iloc[idx]:.3f})m"
        else:
            pos_str = "N/A"
            
        # Calculate segment average velocity
        if last_t_abs is None:
            avg_vel_str = "N/A"
        else:
            mask = (df_mocap['t'] >= last_t_abs) & (df_mocap['t'] <= t_abs)
            if mask.any():
                avg_vel = df_mocap.loc[mask, 'speed'].mean()
                avg_vel_str = f"{avg_vel:.3f} m/s"
            else:
                avg_vel_str = "0.000 m/s"
                
        events_log.append({
            'Event Name': name,
            'Absolute Time (s)': f"{t_abs:.2f}s",
            'Time Since Arming (s)': f"{t_arm_rel:+.2f}s" if t_abs >= arming_time else "N/A",
            'Average Velocity': avg_vel_str,
            'Battery Remaining': bat_pct,
            'Voltage (V)': bat_v,
            'Mocap Coordinate (ENU)': pos_str
        })
        last_t_abs = t_abs

    if not df_mocap.empty:
        add_event("1. Log Start", df_mocap['t'].iloc[0])
    if arming_time is not None and arming_time > 0:
        add_event("2. System Armed", arming_time)
    if takeoff_time is not None:
        add_event("3. Takeoff Detected", takeoff_time)
    
    # Sort waypoint event keys chronologically by their timestamps.
    # This guarantees correct chronological order even though wp_events
    # is a dict (insertion-ordered in Python 3.7+ but not guaranteed
    # to be chronological since detection order may differ).
    sorted_wp_names = sorted(wp_events.keys(), key=lambda k: wp_events[k])
    for wp in sorted_wp_names:
        if wp == 'Column Impact':
            angle_str = f" (Achieved Angle: {achieved_angle:.1f}°)" if achieved_angle is not None else ""
            add_event(f"💥 Column Impact{angle_str}", wp_events[wp])
        elif wp == 'Column Passed':
            add_event("🟢 Column Center Passed", wp_events[wp])
        else:
            add_event(f"4. Arrived {wp}", wp_events[wp])
            
    if disarming_time is not None:
        add_event("5. Disarmed / Landed", disarming_time)
    if not df_mocap.empty:
        add_event("6. Log End", df_mocap['t'].iloc[-1])

    return events_log

def perpendicular_distance(p, p1, p2):
    """Perpendicular distance from a point to the infinite line through p1, p2.

    **Formula (Point-to-Line Distance, 2D):**

      Given point p = (x₀, y₀) and line defined by p₁ = (x₁, y₁), p₂ = (x₂, y₂):

        d = |(y₂ − y₁)·x₀ − (x₂ − x₁)·y₀ + x₂·y₁ − y₂·x₁| / √((y₂ − y₁)² + (x₂ − x₁)²)

    **Derivation:** The numerator is the magnitude of the 2D cross product
    |(p₂ − p₁) × (p − p₁)| = |(x₂−x₁)(y₀−y₁) − (y₂−y₁)(x₀−x₁)|. The denominator is
    the segment length ‖p₂ − p₁‖, normalizing to a perpendicular distance.

    **Standard name:** Point-to-line distance (determinant form).

    **Interaction:** Called by `compute_flight_metrics()` for every MoCap sample in the
    WP2→WP3 transit window. The resulting dᵢ values feed into:
      - `max_tracking_error`    = max(dᵢ)
      - `mean_tracking_error`   = mean(dᵢ)
      - `path_spread_rmsld`     = √(mean(dᵢ²))     [RMSLD]
      - `recovery_area`         = ∫ d(s) ds         [SIAE, trapezoidal]

    Note: This computes distance to the *infinite* line, not the finite segment.
    For this experiment, the drone always lies within the WP2→WP3 longitudinal
    extent, so the distinction is irrelevant.
    """
    x0, y0 = p
    x1, y1 = p1
    x2, y2 = p2
    # Numerator: 2D cross product magnitude = |(p2−p1) × (p−p1)|
    num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    # Denominator: Euclidean length of the line segment
    den = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return num / den if den > 0 else 0.0

def compute_flight_metrics(df_mocap, wp_events, column_x, column_y, column_radius, cage_radius, df_imu=None, df_ekf_kin=None):
    r"""Computes all key collision and trajectory performance metrics for one flight.

    This is the master metrics aggregation function for the entire experimental
    pipeline. It is called by `db_pipeline.py` once per flight and its output
    dict is inserted directly into the SQLite `flights_summary` table by
    `db_manager.insert_or_replace_flight()`.

    ────────────────────────────────────────────────────────────────────────────
    CATEGORY 1 — GEOMETRIC CLEARANCE (always from MoCap position)
    ────────────────────────────────────────────────────────────────────────────

    **Collision distance:**
      d_centre(t) = √((x(t) − x_col)² + (y(t) − y_col)²)

      d_min  = min_t d_centre(t)                                     [metres]

    **Obstacle clearance:**
      c = d_min − r_column − r_cage                                  [metres]

      c < 0  →  physical contact occurred (drone centre entered the
                sum-of-radii boundary).
      c > 0  →  near miss (drone passed without touching the column).

    ────────────────────────────────────────────────────────────────────────────
    CATEGORY 2 — KINEMATICS (velocity/acceleration)
    ────────────────────────────────────────────────────────────────────────────

    Source: EKF vehicle_odometry when `df_ekf_kin` is provided (default since
    2026-06-10); MoCap SG-differentiated velocity otherwise (legacy path).

    **Impact speed:**
      v_impact = ‖v(t_closest)‖ = √(vx² + vy² + vz²)                [m/s]

    **Impact deceleration:**
      a_impact = a(t_closest)                                        [m/s²]
      (scalar acceleration at the instant of minimum column clearance)

    **Pre-impact deceleration baseline:**
      a_before = mean( a(t) )  for t ∈ [t_closest−0.4, t_closest−0.2]   [m/s²]

    **Impact angle** (achieved_impact_angle):
      Let r = vector from drone to column centre: (x_col − x, y_col − y).
      Let v = drone velocity vector at reference time: (vx, vy).

        θ = arccos( (r·v) / (‖r‖·‖v‖) )                             [radians]
        θ_deg = θ × 180/π,  then folded to [0°, 90°]:
          if θ_deg > 90 → θ_deg ← 180 − θ_deg

      Reference time: `Column Impact` event if detected, otherwise `closest_t`.
      Physical meaning: 0° = drone flies directly toward column centre (head-on);
      90° = drone flies tangent to column surface (grazing).

    ────────────────────────────────────────────────────────────────────────────
    CATEGORY 3 — PATH TRACKING (perpendicular deviation from WP2→WP3 line)
    ────────────────────────────────────────────────────────────────────────────

    For each MoCap sample in the WP2→WP3 transit window, the perpendicular
    distance d_i to the commanded straight-line path is computed via
    `perpendicular_distance()` (point-to-line distance, determinant form).

    **Maximum trajectory deviation:**
      d_max = max_i d_i                                               [metres]
      (reported in mm for the master comparison table)

    **Mean trajectory deviation:**
      d_mean = (1/N) Σ d_i                                            [metres]

    **Path spread — RMSLD (Root Mean Square of Lateral Displacement):**
      RMSLD = √( (1/N) Σ d_i² )                                       [metres]
      RMSLD_mm = RMSLD × 1000                                         [mm]

    **Recovery area — SIAE (Spatial Integral of Absolute Error):**
      Let s = along-path distance parameter: s = (p − WP2)·û
      where û = (WP3 − WP2) / ‖WP3 − WP2‖ is the unit direction vector.

        SIAE = ∫_{s_start}^{s_end} |d(s)| ds                         [m²]

      Computed by trapezoidal integration (np.trapz) over the recovery
      segment [t_collision, t_WP3], sorted by s for monotonic integration.
      Reported in mm·m (×1000). Units reduce to metres after normalisation
      to waypoint segment length.

    ────────────────────────────────────────────────────────────────────────────
    CATEGORY 4 — IMU STRUCTURAL RESPONSE
    ────────────────────────────────────────────────────────────────────────────

    **Contact window:** [t_impact − 0.05, t_impact + 0.35]  (400 ms total)

    **Peak values** (per-axis, over contact window):
      Accel:  peak_accel_c = max |a_c(t)|                            [m/s²]
      Gyro:   peak_gyro_c  = max |ω_c(t)|                            [rad/s]
      for c ∈ {x, y, z}.
      Z-acceleration has +g (9.81 m/s²) added back to remove gravity
      compensation, giving total upward acceleration in ENU frame.

    **Integrated energy** (per-axis, over contact window):
      Energy_accel_c = ∫ |a_c(t)| dt                                  [m/s]
      Energy_gyro_c  = ∫ |ω_c(t)| dt                                  [rad]
      using trapezoidal integration (np.trapz). The |·| absolute value
      prevents positive/negative oscillation from cancelling in the
      integral, giving total impulse magnitude per channel.

    **Post-impact standard deviation** (imu_std_*):
      σ_c = std( a_c(t) )  for t ∈ [t_impact+0.2, t_impact+3.0]      [m/s²]
      σ_gc = std( ω_c(t) ) for t ∈ [t_impact+0.2, t_impact+3.0]      [rad/s]
      The first 200 ms are skipped to isolate the vibration tail from
      the initial shock spike. Lower σ values indicate faster vibrational
      settling after impact.

    **Impact/Regular spread** (per-axis, normalised by g):
      σ_impact,c  = std( a_c(t) ) / g   over [t_impact−0.05, t_impact+0.35]
      σ_regular,c = std( a_c(t) ) / g   over [t_impact−1.05, t_impact−0.05]
      The regular window is a 1 s pre-impact baseline representing normal
      flight vibration. The ratio σ_impact/σ_regular indicates how many
      times normal vibration the collision produced.

    **Settling time:**
      t_settle = max{ t : |a_dev(t)| ≥ 1.5 m/s²  ∨  |ω_mag(t)| ≥ 0.5 rad/s }
      Elapsed time from impact until the *last* sample exceeding either
      threshold — the point at which post-impact vibration returns to the
      nominal flight noise floor.

    Parameters
    ----------
    df_ekf_kin : pd.DataFrame or None
        EKF kinematics DataFrame from compute_ekf_kinematics(). When provided,
        velocity/acceleration columns are replaced with EKF values via np.interp
        onto the MoCap time grid. Default since 2026-06-10.
    """
    if df_mocap.empty:
        return {
            'closest_t': 0.0,
            'min_dist_center': 0.0,
            'closest_clearance': 0.0,
            'avg_speed_wp2_wp3': 0.0,
            'max_tracking_error': 0.0,
            'mean_tracking_error': 0.0,
            'max_lateral_displacement': 0.0,
            'imu_peak_accel': None, 'imu_peak_accel_x': None, 'imu_peak_accel_y': None, 'imu_peak_accel_z': None,
            'imu_peak_gyro': None, 'imu_peak_gyro_x': None, 'imu_peak_gyro_y': None, 'imu_peak_gyro_z': None,
            'imu_accel_energy': None, 'imu_accel_energy_x': None, 'imu_accel_energy_y': None, 'imu_accel_energy_z': None,
            'imu_gyro_energy': None, 'imu_gyro_energy_x': None, 'imu_gyro_energy_y': None, 'imu_gyro_energy_z': None,
            'imu_accel_settling': None, 'imu_gyro_settling': None
        }

    # Segment from WP1 to WP4 (active sweep)
    t_start = wp_events.get('WP1', df_mocap['t'].iloc[0])
    t_end = wp_events.get('WP4', df_mocap['t'].iloc[-1])
    df_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]
    if df_sweep.empty:
        df_sweep = df_mocap

    # ==================================================================
    # EKF VELOCITY OVERRIDE MECHANISM
    # ==================================================================
    # When df_ekf_kin is provided, we replace ALL MoCap-derived velocity
    # and acceleration columns (vx, vy, vz, speed, ax, ay, az, accel) with
    # EKF-based values interpolated onto the MoCap time grid via np.interp.
    #
    # This is a surgical column-level replacement: MoCap position (x, y, z)
    # is preserved for geometric calculations (clearance, tracking error),
    # while kinematics (velocity, acceleration) come from the EKF which is
    # inherently smooth even during MoCap dropouts.
    #
    # NaN edges (where EKF and MoCap time grids don't perfectly overlap)
    # are forward/backward filled to prevent downstream NaN propagation.
    # ==================================================================
    if df_ekf_kin is not None and not df_ekf_kin.empty:
        ekf_t = df_ekf_kin['t'].values
        mocap_t = df_mocap['t'].values
        for col in ['vx', 'vy', 'vz', 'speed', 'ax', 'ay', 'az', 'accel']:
            if col in df_ekf_kin.columns:
                df_mocap[col] = np.interp(mocap_t, ekf_t, df_ekf_kin[col].values,
                                          left=np.nan, right=np.nan)
        # Forward/backward fill NaN edges where time grids don't overlap
        df_mocap = df_mocap.ffill().bfill()
        # Re-compute time-sliced DataFrames with the EKF-updated columns
        df_sweep = df_mocap[(df_mocap['t'] >= t_start) & (df_mocap['t'] <= t_end)]
        if df_sweep.empty:
            df_sweep = df_mocap
    # === RETIRED: MoCap-derived velocity (used when df_ekf_kin=None) ===
    # Original pipeline: compute_mocap_kinematics() SG-differentiates MoCap positions.
    # Works well for Rotating Cage (clean 120 Hz MoCap) but produces
    # dropout kinks for Fixed Cage (as low as ~10 Hz).

    # ==================================================================
    # GEOMETRIC CLEARANCE (always from MoCap position)
    # ==================================================================
    # Minimum 2D Euclidean distance from drone (x,y) to column center.
    # Clearance = distance - column_radius - cage_radius.
    # Negative clearance = physical overlap = collision occurred.
    # ==================================================================
    dist_to_col_center = np.sqrt((df_sweep['x'] - column_x)**2 + (df_sweep['y'] - column_y)**2)
    min_idx = np.argmin(dist_to_col_center)
    closest_t = df_sweep['t'].iloc[min_idx]
    min_dist_center = dist_to_col_center.iloc[min_idx]
    closest_clearance = min_dist_center - column_radius - cage_radius

    # Segment from WP2 to WP3 (column transit sweep)
    t_wp2 = wp_events.get('WP2', t_start)
    t_wp3 = wp_events.get('WP3', t_end)
    df_wp2_wp3 = df_mocap[(df_mocap['t'] >= t_wp2) & (df_mocap['t'] <= t_wp3)]
    if df_wp2_wp3.empty:
        df_wp2_wp3 = df_sweep

    avg_speed = df_wp2_wp3['speed'].mean() if not df_wp2_wp3.empty else 0.0

    # Get WP coordinates or fallbacks
    def get_wp_coords(wp_name, fallback):
        t_wp = wp_events.get(wp_name)
        if t_wp is not None:
            idx = np.searchsorted(df_mocap['t'], t_wp)
            idx = min(max(0, idx), len(df_mocap) - 1)
            return df_mocap['x'].iloc[idx], df_mocap['y'].iloc[idx]
        return fallback

    wp2_pos = get_wp_coords('WP2', (0.100, 1.200))
    wp3_pos = get_wp_coords('WP3', (0.100, -1.200))

    # ==================================================================
    # PATH TRACKING: Perpendicular deviation from ideal WP2->WP3 line
    # ==================================================================
    # For each MoCap sample (x_i, y_i) in the WP2→WP3 transit segment,
    # compute the perpendicular distance d_i from the point to the
    # infinite line through WP2 and WP3:
    #
    #   d_i = |cross(P2P3_vector, (x_i - x_wp2, y_i - y_wp2))| / |P2P3_vector|
    #
    #   RMSLD (Root Mean Square of Lateral Displacement):
    #     RMSLD = sqrt( (1/N) * Σ(d_i²) )          [metres]
    #
    #   Converted to mm for reporting:  RMSLD_mm = RMSLD × 1000
    #
    # Unlike maximum deviation (which captures a single extreme point),
    # RMSLD quantifies how tightly the drone tracks the commanded
    # straight-line trajectory over the ENTIRE obstacle passage.
    # ==================================================================
    if not df_wp2_wp3.empty:
        errors = np.array([perpendicular_distance((row['x'], row['y']), wp2_pos, wp3_pos) for _, row in df_wp2_wp3.iterrows()])
        mean_err = float(np.mean(errors))
        max_err = float(np.max(errors))
        # Root Mean Square of Lateral Displacement (mm)
        rmsld = float(np.sqrt(np.mean(errors**2)) * 1000.0)
    else:
        mean_err = 0.0
        max_err = 0.0
        rmsld = 0.0

    # Calculate speed, accel, and before_impact_accel at closest approach (always available)
    achieved_angle = None
    impact_speed = None
    impact_accel = None
    before_impact_accel = None
    
    if not df_mocap.empty:
        # Always compute speed and accel at closest_t (the point of minimum clearance)
        idx_closest = (df_mocap['t'] - closest_t).abs().idxmin()
        closest_row = df_mocap.iloc[idx_closest]
        impact_speed = closest_row.get('speed', 0.0)
        impact_accel = closest_row.get('accel', 0.0)
        
        # before_impact_accel: average accel in window [closest_t - 0.4, closest_t - 0.2]
        window_before = df_mocap[(df_mocap['t'] >= closest_t - 0.4) & (df_mocap['t'] <= closest_t - 0.2)]
        if not window_before.empty:
            before_impact_accel = window_before['accel'].mean()
    
    # ==================================================================
    # IMPACT ANGLE: Angle between collision normal and velocity vector
    # ==================================================================
    # Computed as arccos of the dot product between:
    #   r = vector from drone TO column center (collision normal)
    #   v = drone velocity vector at the impact reference time
    #
    # angle = arccos((rx*vx + ry*vy) / (|r| * |v|))
    #
    # Folded to [0, 90] degrees: a 75-degree collision and a 105-degree
    # collision are physically equivalent (both are 15 degrees from
    # perpendicular). The folding is achieved by: if angle > 90, return
    # 180 - angle.
    #
    # Reference time is Column Impact if detected, otherwise closest_t.
    # ==================================================================
    impact_t = wp_events.get('Column Impact')
    ref_t = impact_t if impact_t is not None else closest_t
    if not df_mocap.empty:
        idx_ref = (df_mocap['t'] - ref_t).abs().idxmin()
        ref_row = df_mocap.iloc[idx_ref]
        rx = column_x - ref_row['x']
        ry = column_y - ref_row['y']
        r_len = np.sqrt(rx**2 + ry**2)
        vx = ref_row.get('vx', 0.0)
        vy = ref_row.get('vy', -1.0)
        v_len = np.sqrt(vx**2 + vy**2)
        
        if r_len > 1e-3 and v_len > 1e-3:
            cos_theta = (rx * vx + ry * vy) / (r_len * v_len)
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            achieved_angle = np.degrees(np.arccos(cos_theta))
            if achieved_angle > 90.0:
                achieved_angle = 180.0 - achieved_angle

    # ==================================================================
    # RECOVERY ANALYSIS: Post-impact path deviation
    # ==================================================================
    # The recovery segment spans from Column Impact (or closest approach)
    # to WP3. We quantify how far the drone deviates from the nominal
    # trajectory after the collision and compute the integrated area.
    # ==================================================================
    t_collision = wp_events.get('Column Impact', closest_t)
    df_recovery = df_wp2_wp3[df_wp2_wp3['t'] >= t_collision]
    
    avg_dev_after = 0.0
    max_dev_after = 0.0
    recovery_area = 0.0

    if not df_recovery.empty and len(df_recovery) >= 2:
        # Calculate perpendicular distances for each point in recovery
        d_recovery = [perpendicular_distance((row['x'], row['y']), wp2_pos, wp3_pos) for _, row in df_recovery.iterrows()]
        avg_dev_after = np.mean(d_recovery) * 1000.0  # mm
        max_dev_after = np.max(d_recovery) * 1000.0  # mm
        
        # Recovery area: trapezoidal integration of lateral deviation d(s)
        # along the nominal path parameter s (distance from WP2).
        # The unit direction vector (ux, uy) points from WP2 to WP3.
        dx_line = wp3_pos[0] - wp2_pos[0]
        dy_line = wp3_pos[1] - wp2_pos[1]
        line_len = np.sqrt(dx_line**2 + dy_line**2)
        if line_len > 0:
            ux = dx_line / line_len  # unit vector along nominal path
            uy = dy_line / line_len
            # Scalar projection of each recovery point onto the nominal
            # line: s = (point - WP2) dot unit_direction. This gives the
            # along-path distance measured from WP2.
            s_recovery = []
            for _, row in df_recovery.iterrows():
                ap_x = row['x'] - wp2_pos[0]
                ap_y = row['y'] - wp2_pos[1]
                s = ap_x * ux + ap_y * uy
                s_recovery.append(s)
            
            s_recovery = np.array(s_recovery)
            d_recovery = np.array(d_recovery)
            # Sort by along-path distance s to ensure monotonic integration.
            # Without sorting, np.trapz would use temporal order, which may
            # not be monotonic in s if the drone reverses after collision.
            sort_idx = np.argsort(s_recovery)
            s_sorted = s_recovery[sort_idx]
            d_sorted = d_recovery[sort_idx]
            
            # Trapezoidal integration along nominal path of travel
            recovery_area = float(np.trapz(d_sorted, s_sorted) * 1000.0)  # mm * m
            
    # ==================================================================
    # STRUCTURAL IMU KINEMATICS & ENERGY METRICS
    # ==================================================================
    # The IMU (onboard BMI088 or similar) measures acceleration at 100 Hz.
    # These metrics quantify the physical severity of the collision through
    # the airframe's structural response: peak transient forces, total
    # impulse delivered, settling time, and post-impact vibration spread.
    #
    # All IMU metrics are computed from the contact window:
    #   [t_impact - 0.05, t_impact + 0.35]  (400 ms total)
    # The 50ms pre-impact padding captures the very start of the impulse,
    # and the 350ms post-impact captures the full structural response
    # including the initial transient and the ringing tail.
    # ==================================================================
    imu_metrics = {
        'imu_peak_accel': None, 'imu_peak_accel_x': None, 'imu_peak_accel_y': None, 'imu_peak_accel_z': None,
        'imu_peak_gyro': None, 'imu_peak_gyro_x': None, 'imu_peak_gyro_y': None, 'imu_peak_gyro_z': None,
        'imu_accel_energy': None, 'imu_accel_energy_x': None, 'imu_accel_energy_y': None, 'imu_accel_energy_z': None,
        'imu_gyro_energy': None, 'imu_gyro_energy_x': None, 'imu_gyro_energy_y': None, 'imu_gyro_energy_z': None,
        'imu_accel_settling': None, 'imu_gyro_settling': None,
        'imu_ax_spread_impact': None, 'imu_ay_spread_impact': None, 'imu_az_spread_impact': None,
        'imu_ax_spread_regular': None, 'imu_ay_spread_regular': None, 'imu_az_spread_regular': None
    }

    if df_imu is not None and not df_imu.empty:
        t_impact = wp_events.get('Column Impact', closest_t)
        # Reverted back to MoCap closest-approach alignment as requested by user


        # 1. Slice contact window strictly to [t_impact - 0.05, t_impact + 0.35]
        df_contact = df_imu[(df_imu['t'] >= t_impact - 0.05) & (df_imu['t'] <= t_impact + 0.35)]
        if df_contact.empty:
            df_contact = df_imu
            
        # 1b. Slice regular flight window strictly to [t_impact - 1.05, t_impact - 0.05]
        df_regular = df_imu[(df_imu['t'] >= t_impact - 1.05) & (df_imu['t'] < t_impact - 0.05)]

        # === PEAK METRICS ===
        # Max absolute value over the contact window for each IMU axis.
        # a_deviation = |accel_vector - gravity| (scalar deviation).
        # Z-acceleration has +9.81 m/s^2 added back to remove gravity
        # compensation, giving the total upward acceleration in ENU.
        # Peak values capture the maximum instantaneous force/rotation
        # experienced during the collision transient.
        imu_metrics['imu_peak_accel'] = float(df_contact['a_deviation'].max())
        imu_metrics['imu_peak_accel_x'] = float(df_contact['ax'].abs().max())
        imu_metrics['imu_peak_accel_y'] = float(df_contact['ay'].abs().max())
        imu_metrics['imu_peak_accel_z'] = float((df_contact['az'] + 9.81).abs().max())

        imu_metrics['imu_peak_gyro'] = float(df_contact['g_mag'].max())
        imu_metrics['imu_peak_gyro_x'] = float(df_contact['gx'].abs().max())
        imu_metrics['imu_peak_gyro_y'] = float(df_contact['gy'].abs().max())
        
        # === SPREAD (STANDARD DEVIATION) ===
        # Impact spread: std dev of acceleration over the contact window,
        # normalized by g. Measures the variability of the structural
        # response during the collision.
        # Regular spread: std dev over a 1 s pre-impact baseline window
        # [t_impact - 1.05, t_impact - 0.05], representing normal flight
        # vibration levels. The impact/regular ratio indicates how many
        # times normal vibration the collision produced.
        if not df_contact.empty:
            imu_metrics['imu_ax_spread_impact'] = float(df_contact['ax'].std() / 9.81)
            imu_metrics['imu_ay_spread_impact'] = float(df_contact['ay'].std() / 9.81)
            imu_metrics['imu_az_spread_impact'] = float((df_contact['az'] + 9.81).std() / 9.81)
            
        if not df_regular.empty:
            imu_metrics['imu_ax_spread_regular'] = float(df_regular['ax'].std() / 9.81)
            imu_metrics['imu_ay_spread_regular'] = float(df_regular['ay'].std() / 9.81)
            imu_metrics['imu_az_spread_regular'] = float((df_regular['az'] + 9.81).std() / 9.81)
        imu_metrics['imu_peak_gyro_z'] = float(df_contact['gz'].abs().max())

        # === INTEGRATED ENERGY (TRAPEZOIDAL INTEGRATION) ===
        # Energy = integral(|signal| dt) over the contact window.
        # This is a proxy for the total impulse/collision energy transferred
        # to the airframe. We integrate the ABSOLUTE value so that positive
        # and negative excursions both contribute to the total (oscillatory
        # motion around zero would cancel out with signed integration).
        # np.trapz uses the composite trapezoidal rule which is exact for
        # piecewise-linear data (appropriate for 100 Hz sampled IMU).
        if len(df_contact) >= 2:
            t_vals = df_contact['t'].values
            imu_metrics['imu_accel_energy'] = float(np.trapz(df_contact['a_deviation'].values, t_vals))
            imu_metrics['imu_accel_energy_x'] = float(np.trapz(df_contact['ax'].abs().values, t_vals))
            imu_metrics['imu_accel_energy_y'] = float(np.trapz(df_contact['ay'].abs().values, t_vals))
            imu_metrics['imu_accel_energy_z'] = float(np.trapz((df_contact['az'] + 9.81).abs().values, t_vals))

            imu_metrics['imu_gyro_energy'] = float(np.trapz(df_contact['g_mag'].values, t_vals))
            imu_metrics['imu_gyro_energy_x'] = float(np.trapz(df_contact['gx'].abs().values, t_vals))
            imu_metrics['imu_gyro_energy_y'] = float(np.trapz(df_contact['gy'].abs().values, t_vals))
            imu_metrics['imu_gyro_energy_z'] = float(np.trapz(df_contact['gz'].abs().values, t_vals))
        else:
            imu_metrics['imu_accel_energy'] = 0.0
            imu_metrics['imu_accel_energy_x'] = 0.0
            imu_metrics['imu_accel_energy_y'] = 0.0
            imu_metrics['imu_accel_energy_z'] = 0.0
            imu_metrics['imu_gyro_energy'] = 0.0
            imu_metrics['imu_gyro_energy_x'] = 0.0
            imu_metrics['imu_gyro_energy_y'] = 0.0
            imu_metrics['imu_gyro_energy_z'] = 0.0

        # === SETTLING TIME ===
        # Settling time = elapsed time from t_impact until the LAST sample
        # where the signal exceeds the detection threshold. This captures
        # how long it takes for the airframe to return to quiescence.
        # Thresholds: a_deviation >= 1.5 m/s^2, gyro magnitude >= 0.5 rad/s.
        # Using the LAST (not first) time above threshold ensures we
        # measure the full duration of the structural response, including
        # any late-ringing modes that persist after the main transient.
        df_after = df_imu[df_imu['t'] >= t_impact]
        if not df_after.empty:
            high_accel_times = df_after[df_after['a_deviation'] >= 1.5]['t']
            if not high_accel_times.empty:
                imu_metrics['imu_accel_settling'] = float(high_accel_times.max() - t_impact)
            else:
                imu_metrics['imu_accel_settling'] = 0.0

            high_gyro_times = df_after[df_after['g_mag'] >= 0.5]['t']
            if not high_gyro_times.empty:
                imu_metrics['imu_gyro_settling'] = float(high_gyro_times.max() - t_impact)
            else:
                imu_metrics['imu_gyro_settling'] = 0.0

            # === VIBRATION SPREAD (POST-IMPACT) ===
            # Std dev of each IMU axis over [t_impact + 0.2, t_impact + 3.0].
            # The first 200 ms AFTER impact is deliberately skipped to
            # isolate the decaying vibration tail from the initial shock
            # spike (which dominates the peak metric). This measures the
            # residual structural ringing -- high values indicate the
            # airframe is still oscillating seconds after the collision.
            # Minimum 5 samples required for a meaningful std dev estimate.
            df_vib = df_imu[(df_imu['t'] >= t_impact + 0.2) & (df_imu['t'] <= t_impact + 3.0)]
            if not df_vib.empty and len(df_vib) >= 5:
                imu_metrics['imu_std_ax'] = float(df_vib['ax'].std())
                imu_metrics['imu_std_ay'] = float(df_vib['ay'].std())
                imu_metrics['imu_std_az'] = float((df_vib['az'] + 9.81).std())
                imu_metrics['imu_std_gx'] = float(df_vib['gx'].std())
                imu_metrics['imu_std_gy'] = float(df_vib['gy'].std())
                imu_metrics['imu_std_gz'] = float(df_vib['gz'].std())
            else:
                imu_metrics['imu_std_ax'] = 0.0
                imu_metrics['imu_std_ay'] = 0.0
                imu_metrics['imu_std_az'] = 0.0
                imu_metrics['imu_std_gx'] = 0.0
                imu_metrics['imu_std_gy'] = 0.0
                imu_metrics['imu_std_gz'] = 0.0
        else:
            imu_metrics['imu_accel_settling'] = 0.0
            imu_metrics['imu_gyro_settling'] = 0.0
            imu_metrics['imu_std_ax'] = 0.0
            imu_metrics['imu_std_ay'] = 0.0
            imu_metrics['imu_std_az'] = 0.0
            imu_metrics['imu_std_gx'] = 0.0
            imu_metrics['imu_std_gy'] = 0.0
            imu_metrics['imu_std_gz'] = 0.0

    return {
        'closest_t': closest_t,
        'min_dist_center': min_dist_center,
        'closest_clearance': closest_clearance,
        'avg_speed_wp2_wp3': avg_speed,
        'max_tracking_error': max_err,
        'mean_tracking_error': mean_err,
        'max_lateral_displacement': max_err,
        'path_spread_rmsld': rmsld,
        'achieved_impact_angle': achieved_angle,
        'impact_speed': impact_speed,
        'impact_accel': impact_accel,
        'before_impact_accel': before_impact_accel,
        'avg_dev_after': avg_dev_after,
        'max_dev_after': max_dev_after,
        'recovery_area': recovery_area,

        # 18 new IMU metrics
        'imu_peak_accel': imu_metrics['imu_peak_accel'],
        'imu_peak_accel_x': imu_metrics['imu_peak_accel_x'],
        'imu_peak_accel_y': imu_metrics['imu_peak_accel_y'],
        'imu_peak_accel_z': imu_metrics['imu_peak_accel_z'],
        'imu_peak_gyro': imu_metrics['imu_peak_gyro'],
        'imu_peak_gyro_x': imu_metrics['imu_peak_gyro_x'],
        'imu_peak_gyro_y': imu_metrics['imu_peak_gyro_y'],
        'imu_peak_gyro_z': imu_metrics['imu_peak_gyro_z'],
        'imu_accel_energy': imu_metrics['imu_accel_energy'],
        'imu_accel_energy_x': imu_metrics['imu_accel_energy_x'],
        'imu_accel_energy_y': imu_metrics['imu_accel_energy_y'],
        'imu_accel_energy_z': imu_metrics['imu_accel_energy_z'],
        'imu_gyro_energy': imu_metrics['imu_gyro_energy'],
        'imu_gyro_energy_x': imu_metrics['imu_gyro_energy_x'],
        'imu_gyro_energy_y': imu_metrics['imu_gyro_energy_y'],
        'imu_gyro_energy_z': imu_metrics['imu_gyro_energy_z'],
        'imu_accel_settling': imu_metrics['imu_accel_settling'],
        'imu_gyro_settling': imu_metrics['imu_gyro_settling'],
        'imu_std_ax': imu_metrics.get('imu_std_ax', 0.0),
        'imu_std_ay': imu_metrics.get('imu_std_ay', 0.0),
        'imu_std_az': imu_metrics.get('imu_std_az', 0.0),
        'imu_std_gx': imu_metrics.get('imu_std_gx', 0.0),
        'imu_std_gy': imu_metrics.get('imu_std_gy', 0.0),
        'imu_std_gz': imu_metrics.get('imu_std_gz', 0.0),
        
        # New IMU acceleration spread metrics
        'imu_ax_spread_impact': imu_metrics.get('imu_ax_spread_impact'),
        'imu_ay_spread_impact': imu_metrics.get('imu_ay_spread_impact'),
        'imu_az_spread_impact': imu_metrics.get('imu_az_spread_impact'),
        'imu_ax_spread_regular': imu_metrics.get('imu_ax_spread_regular'),
        'imu_ay_spread_regular': imu_metrics.get('imu_ay_spread_regular'),
        'imu_az_spread_regular': imu_metrics.get('imu_az_spread_regular')
    }
