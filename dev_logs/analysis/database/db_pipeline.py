import glob
import os
import sys
import glob
import numpy as np
import pandas as pd
from IPython.display import display, HTML

from dev_logs.analysis.database.db_loader import load_drone_metadata, load_mcap, build_dataframes
from dev_logs.analysis.kinematics.kin_calculator import compute_mocap_kinematics, compute_ekf_kinematics, find_waypoint_events, build_events_log, compute_flight_metrics
from dev_logs.analysis.kinematics.kin_plot_trajectory import plot_trajectory
from dev_logs.analysis.kinematics.kin_plot_kinematics import plot_velocity_profile, plot_battery_sag, plot_imu_dynamics, plot_imu_xyz_components, plot_ekf_kinetic_profile
from dev_logs.analysis.kinematics.kin_plot_actuators import plot_actuators_and_status, plot_control_allocator_saturation, plot_pid_rate_tracking
from dev_logs.analysis.kinematics.kin_plot_statistics import plot_angle_boxplots
from dev_logs.analysis.database.db_manager import insert_or_replace_flight, get_database_summary_markdown, is_already_cached, get_battery_efficiency_df, APPROVED_CUTOFF, is_approved_flight, is_excluded_pass, apply_exclusion_config_to_db


# ── Battery rate lookup (flight-level, from flights_battery_efficiency) ──────────────
# Per-pass MCAP windows are too short (~10 s) for meaningful battery drain rates;
# the flights_battery_efficiency table provides flight-level rates computed from
# the full-flight takeoff→landing window using unsliced MCAPs.
# We cache the lookup table globally (lazy, loaded once on first call) rather than
# re-querying the DB for every pass, since all passes share the same flight-level
# rates and the DB table does not change during a pipeline run.
_battery_lookup = None


def _get_battery_rates(flight_folder_name):
    """Return (capacity_drain_rate_flying, voltage_drop_rate_flying) for a flight.

    Why flight-level and not per-pass?
    -----------------------------------
    Per-pass MCAP windows are ~10 s. Battery voltage and capacity drift on the
    order of tenths of a volt / few percent over a full flight (several minutes).
    Dividing that drift over a 10 s window would produce physically meaningless
    rates dominated by measurement noise (the ADC noise floor is larger than the
    true signal over such a short interval). The flights_battery_efficiency table
    instead computes drain rates over the full unsliced MCAP's takeoff→landing
    interval, typically 120--180 s, where the cumulative drift is well above the
    noise floor and the resulting per-minute rate is stable and physically plausible.

    Lazily caches the entire lookup table on first call to avoid re-querying the
    SQLite DB for every pass (all passes within a flight share the same rate).
    """
    global _battery_lookup
    if _battery_lookup is None:
        try:
            df = get_battery_efficiency_df()
            if not df.empty:
                # Index by flight_name so lookups are O(1) dict access
                _battery_lookup = df.set_index('flight_name').to_dict('index')
            else:
                _battery_lookup = {}
        except Exception:
            _battery_lookup = {}
    entry = _battery_lookup.get(flight_folder_name, {})
    return (
        entry.get('capacity_drain_rate_flying'),
        entry.get('voltage_drop_rate_flying'),
    )


def get_ulog_motor_metrics(flight_dir, pass_path, bag_start_ns, wp_events):
    """Extract per-motor control, allocator saturation, and PID tracking metrics
    from the PX4 ULog file, aligned to the MCAP time reference via clock sync.

    Clock synchronization strategy (two-tier fallback)
    --------------------------------------------------
    The MCAP and ULog file are recorded by different clocks (ROS host wall-clock
    vs PX4 µs monotonic on the flight controller). We must estimate the offset
    between them before any cross-file analysis is possible.

    Tier 1 — timesync_status observed_offset (precise, O(~1 µs) accuracy)
        PX4 publishes `/fmu/out/timesync_status` which contains the
        `observed_offset` field — the computed clock offset between the on-board
        companion computer (ROS) and the flight controller (PX4) in microseconds.
        This is a NTP-like round-trip-time protocol running over MAVLink/RTPS
        and is the gold-standard sync source when available.
        We negate it (PX4 time = ROS time - offset) and convert µs → seconds.

    Tier 2 — EKF velocity correlation fallback (heuristic, O(~1 ms) accuracy)
        When timesync_status is missing from the MCAP, we pick one
        vehicle_odometry message from the MCAP (ROS time-stamped) and scan the
        entire ULog vehicle_local_position timeline to find the sample whose
        (vx, vy, vz) velocity vector has the smallest Euclidean distance to the
        MCAP message's velocity. The offset is then:
            offset_sec = MCAP_log_time - ULog_timestamp
        This works because at 10 m/s the drone moves <10 mm per 1 ms — the
        velocity signal changes slowly enough that the nearest-neighbour match
        is unambiguous.
        We reject the match if min_err > 0.01 (m/s)^2 (drift guard).

    Once offset_sec is known, `ulog_to_rel_sec(t_us)` converts a PX4 µs
    timestamp into seconds relative to bag_start_ns (MCAP time origin):
        t_rel = t_us*1e-6 + offset_sec - bag_start_ns*1e-9
    This is the canonical time axis for all ULog-derived metrics below.
    """
    import glob
    import os
    import numpy as np
    import pyulog
    from mcap_ros2.reader import read_ros2_messages

    # Initialize empty metrics — every field defaults to None so that missing
    # ULog topics produce well-defined NULLs in the SQLite DB rather than
    # crashing the pipeline.
    motor_metrics = {
        'motor_avg_before': None,
        'motor_max_before': None,
        'motor_avg_after': None,
        'motor_max_after': None,
        'motor_thrust_surge': None,
        'motor_imbalance_after': None,
        'e_sp_timestamp_PX4': None,
        'e_sp_timestamp_PX4_forw': None,
        'e_ep_timestamp_PX4': None,
        'e_impact_timestamp_PX4': None,
        'max_actuator_output': None
    }

    ulg_files = glob.glob(os.path.join(flight_dir, "*.ulg"))
    if not ulg_files:
        return motor_metrics

    # ── Tier 1: Extract timesync_status observed_offset from MCAP ────────────
    # PX4 publishes a timesync_status message on /fmu/out/timesync_status that
    # contains the computed clock offset (µs) between the ROS companion computer
    # and the flight controller. We search the current pass MCAP first; if not
    # found there (e.g., timesync was published between passes), we fall back to
    # scanning all other MCAPs in the flight directory.
    observed_offset = None
    try:
        # Check current pass MCAP first
        for msg in read_ros2_messages(pass_path):
            if msg.channel.topic == "/fmu/out/timesync_status":
                observed_offset = msg.ros_msg.observed_offset
                break

        # If not in pass, check other MCAPs in directory
        if observed_offset is None:
            mcap_files = glob.glob(os.path.join(flight_dir, "*.mcap"))
            for mf in mcap_files:
                if mf == pass_path:
                    continue
                for msg in read_ros2_messages(mf):
                    if msg.channel.topic == "/fmu/out/timesync_status":
                        observed_offset = msg.ros_msg.observed_offset
                        break
                if observed_offset is not None:
                    break
    except Exception as e:
        print(f"  [WARN] Error reading MCAP for timesync status: {e}")

    # Load ULog
    try:
        ulg_path = ulg_files[0]

        if observed_offset is not None:
            # PX4 timesync reports the ROS→PX4 offset in µs. Our convention:
            # PX4 time = ROS time - offset, so we negate and convert to seconds.
            # Typical values are on the order of ±0.1 ms (stabilised by the
            # timesync convergence loop), which is negligible for our 1-2 s analysis
            # windows but critical for aligning per-sample ULog<->MCAP comparisons.
            offset_sec = - (observed_offset * 1e-6)
            print(f"  ⚡ Clock sync: Using exact timesync offset_sec={offset_sec:.6f}s")
        else:
            # ── Tier 2: EKF velocity correlation fallback ────────────────────
            # Rationale: the EKF velocity estimate (vehicle_odometry in MCAP,
            # vehicle_local_position in ULog) is a continuous, slowly-varying
            # signal — even at 10 m/s the drone moves <10 mm per 1 ms, so the
            # velocity vector at a given instant is essentially a unique
            # fingerprint. We find the ULog sample whose (vx, vy, vz) is closest
            # to the MCAP message's velocity and deduce the clock offset from
            # the timestamp difference.
            # Acceptable error threshold: 0.01 (m/s)^2  (~0.1 m/s per axis).
            # Above this we cannot be confident of a true match.
            print("  ⚠️ Timesync status not found in MCAP. Falling back to EKF velocity correlation...")
            mcap_odom = None
            try:
                for msg in read_ros2_messages(pass_path):
                    if msg.channel.topic == "/fmu/out/vehicle_odometry":
                        mcap_odom = {
                            'ros_ts': msg.ros_msg.timestamp,
                            'log_time_ns': msg.log_time_ns,
                            'vx': msg.ros_msg.velocity[0],
                            'vy': msg.ros_msg.velocity[1],
                            'vz': msg.ros_msg.velocity[2]
                        }
                        break
            except Exception as e:
                print(f"  [WARN] Error reading MCAP for motor sync fallback: {e}")
                return motor_metrics

            if mcap_odom is None:
                print("  [WARN] No /fmu/out/vehicle_odometry found in MCAP for fallback motor sync.")
                return motor_metrics

            ulog_pos = pyulog.ULog(ulg_path, message_name_filter_list=["vehicle_local_position"])
            u_odom = ulog_pos.get_dataset("vehicle_local_position")
            u_timestamps = u_odom.data["timestamp"]
            u_vx = u_odom.data["vx"]
            u_vy = u_odom.data["vy"]
            u_vz = u_odom.data["vz"]

            # Brute-force linear scan to find the ULog sample whose velocity
            # vector has minimum Euclidean distance to the MCAP message.
            # O(n) in the number of ULog samples (~60k for a 2 min flight at
            # 500 Hz), which is fast enough to not warrant a KD-tree or
            # bisection optimisation.
            best_idx = None
            min_err = float('inf')
            for i in range(len(u_timestamps)):
                err = (u_vx[i] - mcap_odom['vx'])**2 + (u_vy[i] - mcap_odom['vy'])**2 + (u_vz[i] - mcap_odom['vz'])**2
                if err < min_err:
                    min_err = err
                    best_idx = i

            # Drift guard: if the closest velocity match still exceeds
            # 0.01 (m/s)^2, the velocity signals are likely from different
            # manoeuvres and the offset would be unreliable.
            if best_idx is None or min_err > 0.01:
                print(f"  [WARN] Failed to find close velocity match in ULog (min_err={min_err:.6e}).")
                return motor_metrics

            # Offset = MCAP log-time (ROS ns) - ULog timestamp (PX4 µs), both
            # converted to seconds. Positive means ULog leads MCAP.
            offset_sec = (mcap_odom['log_time_ns'] * 1e-9) - (u_timestamps[best_idx] * 1e-6)
            print(f"  ⚡ Clock sync: Using EKF velocity correlation offset_sec={offset_sec:.6f}s (min_err={min_err:.3e})")


        # ── Canonical time conversion ────────────────────────────────────────
        # Maps a PX4 µs timestamp to seconds relative to the MCAP bag start.
        #  - t_us * 1e-6       → PX4 seconds (from FC boot)
        #  - + offset_sec       → align to ROS/MCAP wall-clock
        #  - - bag_start_ns*1e-9  → re-reference to bag-start (relative t=0)
        def ulog_to_rel_sec(t_us):
            return (t_us * 1e-6) + offset_sec - (bag_start_ns * 1e-9)

        # Load necessary topics from ULog
        ulog_all = pyulog.ULog(ulg_path, message_name_filter_list=[
            "actuator_motors", "actuator_outputs", "control_allocator_status", 
            "vehicle_rates_setpoint", "vehicle_angular_velocity"
        ])
        motors_data = ulog_all.get_dataset("actuator_motors")
        motor_t_us = motors_data.data["timestamp"]
        motor_t = np.array([ulog_to_rel_sec(t) for t in motor_t_us])

        # Get control commands for the 4 main motors
        c0 = motors_data.data["control[0]"]
        c1 = motors_data.data["control[1]"]
        c2 = motors_data.data["control[2]"]
        c3 = motors_data.data["control[3]"]

        # Average across the 4 main rotors — represents the total thrust
        # command envelope. Individual motors may saturate independently but
        # the mean captures the overall throttle demand.
        motor_avg_signal = (c0 + c1 + c2 + c3) / 4.0

        # ── Analysis windows (relative to impact) ────────────────────────────
        # Impact time is the "Column Impact" detection from waypoint analysis,
        # or falls back to WP2 (experiment start point), or the mid-point of
        # the sweep pass if neither is available.
        t_impact = wp_events.get('Column Impact') or wp_events.get('WP2', None)
        if t_impact is None or np.isnan(t_impact):
            # No impact detected — use the geometric mid-point of the sweep
            # as a proxy reference for window placement.
            t_start = wp_events.get('WP1', 0.0)
            t_end = wp_events.get('WP3', 10.0)
            t_impact = (t_start + t_end) / 2.0

        # Window design rationale:
        #   Before  [-1.0, 0.0]:  steady-state hover/horizontal-sweep thrust,
        #                          averaged to get a clean pre-impact baseline.
        #   After   [ 0.0, 1.0]:  captures the full thrust transient including
        #                          the surge and any secondary oscillation.
        #   Imbal.  [ 0.0, 0.4]:  shorter 400 ms window — the motor imbalance
        #                          spike is a fast transient (20-100 ms) caused
        #                          by the physical impulse of the collision
        #                          twisting the drone; averaging over 400 ms
        #                          captures the peak without diluting it into
        #                          the post-recovery steady state.
        mask_before = (motor_t >= (t_impact - 1.0)) & (motor_t < t_impact)
        mask_after = (motor_t >= t_impact) & (motor_t <= (t_impact + 1.0))
        mask_after_04 = (motor_t >= t_impact) & (motor_t <= (t_impact + 0.4))

        if np.any(mask_before):
            avg_before = float(np.mean(motor_avg_signal[mask_before]))
            max_before = float(np.max(motor_avg_signal[mask_before]))
        else:
            avg_before = None
            max_before = None
            
        try:
            # ── Find the instance of actuator_outputs that actually carries motor data ──
            # PX4 ULogs may contain multiple actuator_outputs instances (e.g., for
            # different actuator groups or multiple FMU I/O channels). We select the
            # instance with the largest peak output[0..3] value — this is the one
            # driving the actual ESC/motors. Silent instances (servo-only or unused
            # PWM groups) stay near their min/mid values and are automatically skipped.
            actuator_output_datasets = [d for d in ulog_all.data_list if d.name == "actuator_outputs"]
            best_actuator_outputs = None
            max_val_found = -1.0

            for inst in actuator_output_datasets:
                vals = [np.max(inst.data[f"output[{j}]"]) for j in range(4) if f"output[{j}]" in inst.data and len(inst.data[f"output[{j}]"]) > 0]
                if vals:
                    inst_max = float(np.max(vals))
                    if inst_max > max_val_found:
                        max_val_found = inst_max
                        best_actuator_outputs = inst

            if best_actuator_outputs is not None:
                # ── PWM → percentage conversion ─────────────────────────────
                # Standard PX4 actuator_outputs use 1000–2000 µs PWM range.
                # We map to 0–100 %:  pct = (PWM - 1000) / 10.0
                #   - 1000 µs =   0 % (motor off / min throttle)
                #   - 1500 µs =  50 % (hover thrust for typical 5" drone)
                #   - 2000 µs = 100 % (full throttle)
                # This linear mapping is the de-facto standard for PX4 PWM
                # outputs when DShot/ESC calibration is not in use.
                motor_metrics['max_actuator_output'] = float((max_val_found - 1000.0) / 10.0)
            else:
                motor_metrics['max_actuator_output'] = None
        except Exception as e:
            print(f"  [WARN] Failed to extract actuator_outputs: {e}")

        if np.any(mask_after):
            avg_after = float(np.mean(motor_avg_signal[mask_after]))
            max_after = float(np.max(motor_avg_signal[mask_after]))
        else:
            avg_after = None
            max_after = None

        if np.any(mask_after_04):
            # ── Motor imbalance metric ───────────────────────────────────────
            # For each time-stamp in the 400 ms post-impact window, compute the
            # standard deviation of the 4 motor control values. The per-sample
            # std dev quantifies how unevenly thrust is distributed across rotors
            # at that instant (perfect symmetry = 0, one motor at max and one at
            # min ≈ 0.5). We then average these per-sample std devs over the
            # window to obtain a single scalar: motor_imbalance_after.
            #
            # Physical interpretation: a collision applies an asymmetric torque
            # impulse to the airframe. The PX4 rate controller counters by
            # differential thrust — one side of the quadrotor spools up while the
            # other spools down. The resulting inter-motor spread is a proxy for
            # the magnitude of the disturbance torque the controller had to fight.
            imbalances = []
            for idx in np.where(mask_after_04)[0]:
                devs = [c0[idx], c1[idx], c2[idx], c3[idx]]
                imbalances.append(np.std(devs))
            imbalance_after = float(np.mean(imbalances))

            # Individual motor averages over the 0.4s window — useful for
            # diagnosing which rotor(s) bore the brunt of the disturbance
            # (e.g., motor closest to the impact side will show elevated control
            # command compared to the opposite motor).
            m1_avg_after = float(np.mean(c0[mask_after_04]))
            m2_avg_after = float(np.mean(c1[mask_after_04]))
            m3_avg_after = float(np.mean(c2[mask_after_04]))
            m4_avg_after = float(np.mean(c3[mask_after_04]))
        else:
            imbalance_after = None
            m1_avg_after = None
            m2_avg_after = None
            m3_avg_after = None
            m4_avg_after = None

        if avg_before is not None and max_after is not None:
            # ── Thrust surge ─────────────────────────────────────────────────
            # Defined as: (peak motor average in 1 s after impact) minus
            # (mean motor average in 1 s before impact).
            # This captures the net throttle spike the flight controller commands
            # in response to the impact — the control system's effort to arrest
            # the post-collision altitude drop and rotational disturbance.
            # Positive = controller demanded more thrust after impact (typical),
            # negative = controller cut thrust (would indicate a pre-planned
            # motor stop or a catastrophic control inversion — not expected).
            thrust_surge = float(max_after - avg_before)
        else:
            thrust_surge = None

        # ── Convert MCAP-relative seconds back to PX4 µs ─────────────────────
        # Inverse of ulog_to_rel_sec:  t_px4_us = (t_rel + bag_start_s - offset_s) * 1e6
        # This allows us to tag ULog data with the PX4 timestamps corresponding
        # to the waypoint events detected in MoCap space.
        def _to_px4_us(t_sec):
            if t_sec is None:
                return None
            return int(round((t_sec - offset_sec + (bag_start_ns * 1e-9)) * 1e6))

        # e_sp = experiment start point (PX4 µs)
        # WP2_cmd is the commanded start (from setpoint instruction, PX4-side),
        # while WP2 is the MoCap-detected forward crossing of the start gate.
        # We prefer the PX4-commanded time when available for closer alignment
        # with the flight controller's internal state machine.
        e_sp_px4      = _to_px4_us(wp_events.get('WP2_cmd') or wp_events.get('WP2'))
        # e_sp_forw = refined forward start (MoCap-detected crossing)
        e_sp_px4_forw = _to_px4_us(wp_events.get('WP2'))
        e_ep_px4      = _to_px4_us(wp_events.get('WP3'))
        e_impact_px4  = _to_px4_us(wp_events.get('Column Impact'))

        # ── 50 ms suppression threshold ─────────────────────────────────────
        # When the PX4-commanded start (WP2_cmd) and the MoCap-detected forward
        # crossing (WP2) differ by less than 50 ms, the two timestamps are
        # effectively co-incident at the resolution of our analysis windows
        # (1-2 s). Duplicating them as separate columns in the DB adds noise
        # without analytical value. Suppressing e_sp_px4_forw keeps the DB
        # schema cleaner: non-NULL e_sp_px4_forw means "there is a meaningful
        # time gap between commanded start and actual forward motion" (e.g.,
        # the drone had to rotate in-place before starting the sweep).
        if e_sp_px4 is not None and e_sp_px4_forw is not None:
            if abs(e_sp_px4_forw - e_sp_px4) < 50000:   # < 50 ms in µs
                e_sp_px4_forw = None

        # Initialize default new metrics
        allocator_saturation_duration_sec = 0.0
        max_unallocated_torque = None
        thrust_setpoint_achieved_pct = None
        roll_rate_error_rms = None
        pitch_rate_error_rms = None
        yaw_rate_error_rms = None

        # ══════════════════════════════════════════════════════════════════════
        # Control allocator saturation duration (3-tier fallback strategy)
        # ══════════════════════════════════════════════════════════════════════
        # Saturation occurs when the control allocator requests more motor
        # thrust than the physical hardware can deliver (e.g., command = 1.0
        # but max thrust = 80% of theoretical). We measure the accumulated
        # wall-clock time any motor spends in saturation during the 1 s
        # post-impact window using a time-weighted sum of inter-sample dt.
        #
        # We try three data sources in priority order, each with different
        # fidelity and availability:
        #
        # Tier 1 — actuator_motors.control[i] (normalised 0..1)
        #   The PX4 mixer output in normalised throttle space, published at
        #   ~10 Hz. This is the "purest" signal — it reflects the allocator's
        #   intent before ESC-specific PWM mapping and before any hardware
        #   clipping. Saturation detected when any motor's control value
        #   reaches ≥0.999 (upper rail) or ≤0.001 (lower rail, i.e., motor
        #   commanded to minimum/stop). The 0.999/0.001 thresholds are used
        #   instead of exactly 1.0/0.0 to account for floating-point rounding
        #   at the ULog encoding boundary.
        #
        # Tier 2 — actuator_outputs.output[i] (PWM µs, 1000–2000)
        #   The raw PWM values written to the ESC hardware, published at the
        #   mixer rate (~250 Hz on PX4). Falls back when actuator_motors is
        #   not logged (some firmware builds omit it). We auto-detect the PWM
        #   range: if max >1500 and min <900, the outputs use the "wide"
        #   range with reversed idle (PWM 115 µs), meaning saturation rails
        #   are 1999 µs (upper) and 115 µs (lower). Otherwise we use standard
        #   1000–2000 µs rails. We select the actuator_outputs instance with
        #   the most samples (some ULogs have multiple, e.g., one per I/O
        #   channel; the motor-driving instance always has the densest data).
        #
        # Tier 3 — control_allocator_status.actuator_saturation[i] (binary flags)
        #   PX4's built-in saturation detection — each actuator gets a flag
        #   (1 = saturated, 0 = not). This is the least granular (binary, not
        #   continuous) but is guaranteed to be published whenever the control
        #   allocator runs (~100 Hz on most configurations). Used as a last
        #   resort when neither actuator_motors nor actuator_outputs is
        #   available in the ULog.
        #
        # Time-weighted sum formula:
        #   saturation_duration = Σ dt_i  for all samples i where saturated
        #   where dt_i = t_{i+1} - t_i (forward difference, with the last
        #   sample's dt replicated). This gives the wall-clock duration in
        #   seconds — a 0.2 s result means the drone spent 200 ms with at
        #   least one motor at its physical limit.
        try:
            # Tier 1: actuator_motors (normalised 0..1, ~10 Hz)
            # The 0.999/0.001 thresholds account for floating-point rounding
            # at the ULog serialisation boundary — a true 1.0 may appear as
            # 0.999999 in the binary payload.
            motors_data = ulog_all.get_dataset("actuator_motors")
            motor_t_us = motors_data.data["timestamp"]
            motor_t = np.array([ulog_to_rel_sec(t) for t in motor_t_us])
            mask_motors = (motor_t >= t_impact) & (motor_t <= (t_impact + 1.0))
            if np.any(mask_motors):
                # Per-axis saturation: any of the 4 motors hitting upper or lower rail
                sat_upper = np.any([motors_data.data[f"control[{i}]"] >= 0.999 for i in range(4)], axis=0)
                sat_lower = np.any([motors_data.data[f"control[{i}]"] <= 0.001 for i in range(4)], axis=0)
                sat_any = sat_upper | sat_lower
                # Forward-difference time deltas for time-weighted accumulation
                motor_dt = np.diff(motor_t)
                motor_dt = np.append(motor_dt, motor_dt[-1] if len(motor_dt) > 0 else 0.0)
                allocator_saturation_duration_sec = float(np.sum(motor_dt[mask_motors & sat_any]))
        except Exception as e_motors:
            print(f"  [WARN] Failed to compute saturation from actuator_motors: {e_motors}")
            # Tier 2: actuator_outputs (PWM µs, ~250 Hz)
            try:
                o_ds = None
                max_pts = -1
                # Find the instance with the most data points — this is the
                # motor-driving channel; other instances are for servo groups
                # or unused I/O and have fewer or zero samples.
                for d in ulog_all.data_list:
                    if d.name == 'actuator_outputs':
                        pts = len(d.data['timestamp'])
                        if pts > max_pts:
                            max_pts = pts
                            o_ds = d
                if o_ds is not None:
                    o_t_us = o_ds.data["timestamp"]
                    o_t = np.array([ulog_to_rel_sec(t) for t in o_t_us])
                    mask_o = (o_t >= t_impact) & (o_t <= (t_impact + 1.0))
                    if np.any(mask_o):
                        o_outputs = [o_ds.data[f"output[{i}]"] for i in range(4)]
                        max_val = max([out.max() for out in o_outputs if len(out) > 0])
                        min_val = min([out.min() for out in o_outputs if len(out) > 0])

                        # Auto-detect PWM range: DShot-capable builds use a
                        # "wide" range where idle is reversed (115 µs) and max
                        # is 1999 µs. Standard PWM builds use 1000–2000 µs.
                        # The heuristic: if any output exceeds 1500 AND any
                        # output dips below 900, we are in wide/reversed-idle
                        # mode. Otherwise, standard 1000–2000.
                        if max_val > 1500 and min_val < 900:
                            upper_limit = 1999.0
                            lower_limit = 115.0
                        else:
                            upper_limit = 2000.0
                            lower_limit = 1000.0

                        o_sat_upper = np.any([o_ds.data[f"output[{i}]"] >= upper_limit for i in range(4)], axis=0)
                        o_sat_lower = np.any([o_ds.data[f"output[{i}]"] <= lower_limit for i in range(4)], axis=0)
                        o_sat_any = o_sat_upper | o_sat_lower

                        o_dt = np.diff(o_t)
                        o_dt = np.append(o_dt, o_dt[-1] if len(o_dt) > 0 else 0.0)
                        allocator_saturation_duration_sec = float(np.sum(o_dt[mask_o & o_sat_any]))
            except Exception as e_outputs:
                print(f"  [WARN] Failed to compute saturation from actuator_outputs: {e_outputs}")
                # Tier 3: control_allocator_status.actuator_saturation (binary flags)
                try:
                    ds_alloc = ulog_all.get_dataset("control_allocator_status")
                    alloc_t_us = ds_alloc.data["timestamp"]
                    alloc_t = np.array([ulog_to_rel_sec(t) for t in alloc_t_us])
                    mask_alloc = (alloc_t >= t_impact) & (alloc_t <= (t_impact + 1.0))
                    if np.any(mask_alloc):
                        sat = [ds_alloc.data[f"actuator_saturation[{i}]"] for i in range(4)]
                        # PX4 stores actuator_saturation as uint8: 0 = not
                        # saturated, 1 (or any non-zero) = saturated.
                        sat_any = np.any([sat[i] != 0 for i in range(4)], axis=0)
                        alloc_dt = np.diff(alloc_t)
                        alloc_dt = np.append(alloc_dt, alloc_dt[-1] if len(alloc_dt) > 0 else 0.0)
                        allocator_saturation_duration_sec = float(np.sum(alloc_dt[mask_alloc & sat_any]))
                except Exception as e_alloc:
                    print(f"  [WARN] Failed to compute saturation fallback: {e_alloc}")

        # ── Other control_allocator_status metrics ───────────────────────────
        # These come from PX4's built-in control allocator diagnostics, which
        # are published at the control loop rate (~100 Hz) regardless of
        # whether actuator_motors or actuator_outputs are logged.
        try:
            ds_alloc = ulog_all.get_dataset("control_allocator_status")
            alloc_t_us = ds_alloc.data["timestamp"]
            alloc_t = np.array([ulog_to_rel_sec(t) for t in alloc_t_us])
            mask_alloc = (alloc_t >= t_impact) & (alloc_t <= (t_impact + 1.0))
            if np.any(mask_alloc):
                # ││Δτ││₂ — Euclidean norm of the 3D unallocated torque vector
                # (roll, pitch, yaw). When the allocator cannot satisfy the
                # full demanded torque, the residual (unallocated) vector
                # quantifies how "under-served" the attitude request is.
                # We take the peak norm in the 1 s post-impact window — a
                # single large value (>0.1 N·m) indicates the controller
                # demanded more torque than the motors could deliver at that
                # instant (saturation event).
                unallocated_torque_norm = np.sqrt(
                    ds_alloc.data["unallocated_torque[0]"]**2 +
                    ds_alloc.data["unallocated_torque[1]"]**2 +
                    ds_alloc.data["unallocated_torque[2]"]**2
                )
                max_unallocated_torque = float(np.max(unallocated_torque_norm[mask_alloc]))
                # thrust_setpoint_achieved ∈ [0, 1] — fraction of the commanded
                # thrust that the allocator was able to assign to motors.
                # Averaged over the 1 s window and scaled to percentage.
                # Values < 100 % indicate thrust saturation (controller asked
                # for more collective thrust than available).
                thrust_setpoint_achieved_pct = float(np.mean(ds_alloc.data["thrust_setpoint_achieved"][mask_alloc]) * 100.0)
        except Exception as e:
            print(f"  [WARN] Failed to compute control allocator status metrics: {e}")

        # ══════════════════════════════════════════════════════════════════════
        # PID rate tracking error (RMS)
        # ══════════════════════════════════════════════════════════════════════
        # Measures how well the PX4 rate controller tracked the angular
        # velocity setpoint during the 1 s post-impact window.
        #
        # Data sources:
        #   - vehicle_rates_setpoint: desired roll/pitch/yaw rate (rad/s),
        #     published at controller loop rate (~1 kHz on PX4).
        #   - vehicle_angular_velocity: measured angular velocity (rad/s),
        #     from gyroscope integration + EKF correction, published at IMU
        #     rate (~1 kHz).
        #
        # These two topics have different timestamps (asynchronous publication
        # schedules). To compute the error we:
        #   1. Resample the setpoint onto the angular velocity timestamps
        #      using np.interp() (linear interpolation). This avoids
        #      timestamp-matching ambiguity and preserves the velocity
        #      signal's native sampling grid (no decimation).
        #   2. Compute error = measured - setpoint at each velocity sample.
        #   3. Compute RMS error = sqrt(mean(error^2)) — the root-mean-square
        #      of the tracking error, which penalises large deviations
        #      quadratically and is the standard metric for evaluating
        #      closed-loop tracking performance.
        #
        # RMS error interpretation:
        #   < 0.05 rad/s  → excellent tracking (normal hover/sweep)
        #   0.05–0.2       → moderate disturbance (wind gust, contact)
        #   > 0.2          → severe tracking loss (collision impulse or
        #                    saturation-induced oscillation)
        try:
            ds_sp = ulog_all.get_dataset("vehicle_rates_setpoint")
            ds_vel = ulog_all.get_dataset("vehicle_angular_velocity")
            sp_t = np.array([ulog_to_rel_sec(t) for t in ds_sp.data["timestamp"]])
            vel_t = np.array([ulog_to_rel_sec(t) for t in ds_vel.data["timestamp"]])
            mask_vel = (vel_t >= t_impact) & (vel_t <= (t_impact + 1.0))
            if np.any(mask_vel):
                vel_t_win = vel_t[mask_vel]
                # ── Timestamp interpolation ──────────────────────────────────
                # np.interp() performs piecewise-linear interpolation of the
                # setpoint (sp_t, sp_val) onto the velocity timestamps vel_t_win.
                # Since the setpoint typically arrives at a slightly different
                # rate (often slower, ~250 Hz vs 1 kHz), this stretches each
                # setpoint value to fill the gaps between velocity samples. The
                # assumption is that the setpoint varies slowly relative to the
                # inter-sample spacing (~1-4 ms), so linear interpolation is
                # accurate to within the setpoint's own rate of change.
                roll_sp_interp = np.interp(vel_t_win, sp_t, ds_sp.data["roll"])
                pitch_sp_interp = np.interp(vel_t_win, sp_t, ds_sp.data["pitch"])
                yaw_sp_interp = np.interp(vel_t_win, sp_t, ds_sp.data["yaw"])

                # Error computation on the velocity sampling grid
                roll_err = ds_vel.data["xyz[0]"][mask_vel] - roll_sp_interp
                pitch_err = ds_vel.data["xyz[1]"][mask_vel] - pitch_sp_interp
                yaw_err = ds_vel.data["xyz[2]"][mask_vel] - yaw_sp_interp

                # ── RMS error ────────────────────────────────────────────────
                # sqrt(mean(err^2)) is the canonical L2 tracking metric — it
                # penalises large errors quadratically (a 0.5 rad/s spike
                # contributes 25x more to the metric than a 0.1 rad/s baseline).
                # This makes it sensitive to the brief, large tracking errors
                # characteristic of collision events.
                roll_rate_error_rms = float(np.sqrt(np.mean(roll_err**2)))
                pitch_rate_error_rms = float(np.sqrt(np.mean(pitch_err**2)))
                yaw_rate_error_rms = float(np.sqrt(np.mean(yaw_err**2)))
        except Exception as e:
            print(f"  [WARN] Failed to compute PID tracking metrics: {e}")

        motor_metrics.update({
            'motor_avg_before': avg_before,
            'motor_max_before': max_before,
            'motor_avg_after': avg_after,
            'motor_max_after': max_after,
            'motor_thrust_surge': thrust_surge,
            'motor_imbalance_after': imbalance_after,
            'motor_m1_avg_after': m1_avg_after,
            'motor_m2_avg_after': m2_avg_after,
            'motor_m3_avg_after': m3_avg_after,
            'motor_m4_avg_after': m4_avg_after,
            'e_sp_timestamp_PX4': e_sp_px4,
            'e_sp_timestamp_PX4_forw': e_sp_px4_forw,
            'e_ep_timestamp_PX4': e_ep_px4,
            'e_impact_timestamp_PX4': e_impact_px4,
            'allocator_saturation_duration_sec': allocator_saturation_duration_sec,
            'max_unallocated_torque': max_unallocated_torque,
            'thrust_setpoint_achieved_pct': thrust_setpoint_achieved_pct,
            'roll_rate_error_rms': roll_rate_error_rms,
            'pitch_rate_error_rms': pitch_rate_error_rms,
            'yaw_rate_error_rms': yaw_rate_error_rms,
            'offset_sec': offset_sec
        })
        if avg_before is not None and max_after is not None and thrust_surge is not None:
            print(f"  ⚡ Motor stats computed: avg_before={avg_before:.3f}, max_after={max_after:.3f}, surge={thrust_surge:.3f}")
        else:
            print(f"  ⚡ Motor stats: Not available (avg_before={avg_before}, max_after={max_after}, surge={thrust_surge})")
    except Exception as e:
        print(f"  [WARN] Failed to process ULog motor data: {e}")

    return motor_metrics


def run(label, angle_deg, column_x=0.408, column_y=0.358, 
        flights_rotating_cage=None, flights_fixed_cage=None, 
        representative_rotating_cage=0, representative_fixed_cage=0, 
        project_root=None, **kwargs):
    """Orchestrates the loading, analysis, and visualization pipeline for one angle.
    Processes both rotating cage and fixed cage passes, prints event tables, and draws
    representative plots and comparative statistics boxplots.
    """
    # Map old aliases if provided for robustness
    if flights_rotating_cage is None:
        flights_rotating_cage = kwargs.get('flights_cage', None)
    if flights_fixed_cage is None:
        flights_fixed_cage = kwargs.get('flights_no_cage', None)

    if flights_rotating_cage is None:
        flights_rotating_cage = []
    if flights_fixed_cage is None:
        flights_fixed_cage = []

    representative_rotating_cage = kwargs.get('representative_cage', representative_rotating_cage)
    representative_fixed_cage = kwargs.get('representative_no_cage', representative_fixed_cage)

    # 1. Resolve project root and flights path dynamically
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if project_root is None:
        project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

    flights_dir = os.path.join(project_root, "dev_logs", "flights")

    # Load SSoT drone configurations
    drone_tracker_name, system_config = load_drone_metadata(project_root)
    primary_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "primary"), {})
    obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})
    
    cage_diameter = primary_body.get("cage_diameter_m", 0.358)
    column_diameter = obstacle_body.get("diameter_m", 0.09)
    cage_radius = cage_diameter / 2.0
    column_radius = column_diameter / 2.0

    print(f"🎬 Starting Analysis Pipeline for: {label} ({angle_deg}°)")
    print(f"🔧 Loaded SSoT Configs: Tracker='{drone_tracker_name}', Cage D={cage_diameter*100:.1f}cm, Column D={column_diameter*100:.1f}cm\n")

    # Helper to process a list of flight folders
    def process_flights(flight_folder_names, condition_label, force_plot=False):
        metrics_list = []
        details_list = []
        
        import glob
        import re

        pass_files_to_process = []
        for f_name in flight_folder_names:
            m_pass = re.search(r'^(.*?) - Pass-(\d+)$', f_name)
            if m_pass:
                base_folder = m_pass.group(1)
                pass_idx = int(m_pass.group(2))
                
                dir_path = os.path.join(flights_dir, base_folder)
                if not os.path.exists(dir_path):
                    dir_path = os.path.join(project_root, base_folder)
                
                pattern = os.path.join(dir_path, f"*-pass{pass_idx:02d}.mcap")
                matches = glob.glob(pattern)
                if matches:
                    pass_files_to_process.append((base_folder, pass_idx, matches[0]))
                else:
                    print(f"[WARN] Pass file not found for: {f_name}")
            else:
                base_folder = f_name
                dir_path = os.path.join(flights_dir, base_folder)
                if not os.path.exists(dir_path):
                    dir_path = os.path.join(project_root, base_folder)
                
                pattern = os.path.join(dir_path, "*-pass*.mcap")
                matches = sorted(glob.glob(pattern))
                if matches:
                    for m_path in matches:
                        m = re.search(r'-pass(\d+)\.mcap$', os.path.basename(m_path))
                        if m:
                            p_idx = int(m.group(1))
                            pass_files_to_process.append((base_folder, p_idx, m_path))
                else:
                    print(f"[WARN] No pass files found in directory: {f_name}")

        total_in_flight = len(pass_files_to_process)
        for pi, (base_folder, pass_idx, pass_path) in enumerate(pass_files_to_process, 1):
            try:
                # Load and process MCAP
                topic_data, bag_start_ns = load_mcap(pass_path)
                dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)
                
                df_mocap = dfs['mocap']
                df_setpoint = dfs.get('setpoint', pd.DataFrame())
                df_bat = dfs['battery']
                df_imu = dfs.get('imu', pd.DataFrame())
                df_column = dfs.get('column', pd.DataFrame())
                arming_time = dfs['arming_time']
                disarming_time = dfs.get('disarming_time')
                if disarming_time is None:
                    disarming_time = arming_time + 10.0
                mocap_rate = dfs.get('mocap_rate', 240.0)
                dynamic_waypoints = dfs.get('dynamic_waypoints', [])
                
                # Compute both raw and smoothed velocity derivative signals from MoCap.
                # Raw: velocity computed from raw MoCap positions via Savitzky-Golay
                # (no resampling) — preserves original time spacing, useful for
                # debugging MoCap dropout artefacts.
                # Smoothed: resampled to uniform grid before S-G differentiation —
                # produces smoother derivatives, used for waypoint detection.
                df_mocap_raw = compute_mocap_kinematics(df_mocap.copy(), resample=False)
                df_mocap = compute_mocap_kinematics(df_mocap, resample=True)

                # === ENABLED: EKF Kinematics (2026-06-09) ===
                # Compute velocity/acceleration from PX4 EKF odometry (vehicle_odometry).
                # Architecture:
                #   The PX4 Extended Kalman Filter fuses MoCap pose measurements
                #   (at 120 Hz for Rotating Cage / ~10 Hz for Fixed Cage due to
                #   optical dropout) with the onboard IMU (gyro + accel at 1 kHz).
                #   This fusion runs at 100–250 Hz on the flight controller and
                #   produces a state estimate that is:
                #     - Smooth (IMU dead-reckoning bridges MoCap gaps) even during
                #       Fixed Cage optical blackouts where MoCap drops to ~10 Hz.
                #     - Drift-free (MoCap corrections bound the IMU integration).
                #   The resulting velocity signal is inherently smoother than pure
                #   MoCap S-G differentiation because the EKF internally applies
                #   a kinematic model with IMU propagation, rather than a purely
                #   geometric filter (S-G) that has no physics model.
                #
                #   Passed to compute_flight_metrics() as df_ekf_kin, which uses it for
                #   impact speed, acceleration, and tangential acceleration metrics
                #   in preference to the MoCap-derived velocities.
                df_odom = dfs.get('odom', pd.DataFrame())
                df_ekf_kin = compute_ekf_kinematics(df_odom, df_mocap)
                if df_ekf_kin is not None:
                    print(f"  ⚡ EKF kinematics computed: {len(df_ekf_kin)} samples @ 100 Hz")
                else:
                    print(f"  ⚡ EKF kinematics: not available (no vehicle_odometry data)")
                # === RETIRED: MoCap-only velocity ===
                # Previously, all velocity/accel came from SG differentiation of MoCap
                # positions. Works for Rotating Cage (120 Hz) but produces dropout kinks
                # for Fixed Cage (~10 Hz). EKF replaces this when available.

                # ── Dynamic column position detection ────────────────────────
                # The obstacle column may be placed at slightly different
                # positions in each flight (small setup variations). Instead of
                # relying on a hardcoded nominal XY, we detect the actual column
                # position from the first 50 MoCap samples.
                #
                # Why the mean of the first 50 samples?
                #   The column is a static rigid body tracked by the MoCap
                #   system. Its pose is published at MoCap rate (120 Hz) from
                #   the moment the system is armed. The first 50 samples (~0.4 s)
                #   provide a reliable static position estimate before the drone
                #   takes off. Averaging reduces single-frame MoCap jitter
                #   (typically ±1-2 mm for a rigid body).
                if df_column is not None and not df_column.empty:
                    col_x_flight = df_column['x'].head(50).mean()
                    col_y_flight = df_column['y'].head(50).mean()
                    print(f"📍 [{base_folder}] Detected Live Column Position: ({col_x_flight:.3f}, {col_y_flight:.3f})")
                else:
                    col_x_flight = column_x
                    col_y_flight = column_y
                    print(f"📍 [{base_folder}] Column not found in poses. Falling back to default: ({col_x_flight:.3f}, {col_y_flight:.3f})")

                # ── Takeoff detection ────────────────────────────────────────
                # Simple Z-height threshold: the drone is considered airborne
                # when its MoCap Z coordinate exceeds 0.15 m (150 mm) above the
                # ground plane. The 0.15 m threshold is chosen because:
                #   - It is well above ground-effect altitude (~50 mm for a
                #     5-inch prop at takeoff RPM).
                #   - It is high enough to reject MoCap Z jitter (<5 mm RMS for
                #     a well-calibrated OptiTrack system) while low enough to
                #     capture the beginning of the climb phase.
                #   - If the drone never crosses 0.15 m (e.g., aborted flight),
                #     we fall back to arming_time + 2 s as a conservative estimate.
                takeoff_mask = df_mocap['z'] > 0.15 if not df_mocap.empty else pd.Series(dtype=bool)
                takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else arming_time + 2.0
                
                # Detect waypoint events with dynamic column Y coordinate and impact detection (get ALL passes)
                wp_events_list = find_waypoint_events(
                    df_mocap, df_setpoint, takeoff_time, label=f"{base_folder} - Pass-{pass_idx:02d}", 
                    column_x=col_x_flight, column_y=col_y_flight, 
                    column_radius=column_radius, cage_radius=cage_radius,
                    return_all=True, dynamic_waypoints=dynamic_waypoints
                )
                
                if not wp_events_list:
                    wp_events_list = [{}]
 
                # --- Resolve declared sweep_speed from the mission class ---
                sweep_speed = None
                try:
                    from dev_logs.analysis.kinematics.kin_calculator import detect_mission_class
                    _m = detect_mission_class(df_setpoint, label=base_folder)
                    sweep_speed = _m.sweep_speed  # e.g. 0.3 m/s
                except Exception:
                    pass  # Not critical — falls back to None in the DB
 
                for p_idx, wp_events in enumerate(wp_events_list):
                    pass_name = f"{base_folder} - Pass-{pass_idx:02d}"

                    # ── Exclusion gate: skip passes manually reviewed as invalid ──
                    # Some passes are manually flagged in the database as invalid
                    # (e.g., collision with safety net instead of column, GPS
                    # glitch causing RTL mid-pass, experimenter walked into MoCap
                    # volume). These are stored in the DB exclusion table and
                    # are skipped entirely — no metrics, no plots, no DB row.
                    # This gate ensures contaminated data never enters the
                    # analysis dataset. The exclusion list is maintained by
                    # manual review of trajectory plots + video footage.
                    if is_excluded_pass(pass_name):
                        from dev_logs.analysis.database.db_manager import get_exclusion_info
                        _, reason, _ = get_exclusion_info(pass_name)
                        print(f"[{pi}/{total_in_flight}] ⏭️  '{pass_name}' excluded by manual review ({reason}), skipping.")
                        continue

                    # Only write a complete pass (WP1 + WP2 + WP3 all detected)
                    has_full_pass = ('WP1' in wp_events or 'WP2' in wp_events) and 'WP3' in wp_events
                    if not has_full_pass:
                        print(f"[{pi}/{total_in_flight}] ⚠️  Incomplete pass detected ({pass_name}), skipping DB insert.")
                        continue

                    print(f"[{pi}/{total_in_flight}] 🔄 Segmented implicit pass: {pass_name}")

                    # Calculate physical metrics with dynamic column coordinates
                    # EKF kinematics replaces MoCap-derived velocity for the metric computation
                    metrics = compute_flight_metrics(df_mocap, wp_events, col_x_flight, col_y_flight, column_radius, cage_radius, df_imu=df_imu, df_ekf_kin=df_ekf_kin)
                    closest_clearance = metrics.get('closest_clearance')
                    metrics['impact_detected'] = 1 if (closest_clearance is not None and closest_clearance < 0.0) else 0

                    # Read declared nominal waypoints from mission SSoT
                    try:
                        from dev_logs.analysis.kinematics.kin_calculator import detect_mission_class
                        _m_dyn = detect_mission_class(df_setpoint, label=base_folder)
                        nom_sp = (_m_dyn.exp_sp[0], _m_dyn.exp_sp[1], _m_dyn.exp_sp[2])
                        nom_ep = (_m_dyn.exp_ep[0], _m_dyn.exp_ep[1], _m_dyn.exp_ep[2])
                    except Exception:
                        is_45 = base_folder and "45" in base_folder.lower()
                        x_lane = 0.248 if is_45 else 0.186
                        nom_sp = (x_lane, 0.950, 0.500)
                        nom_ep = (x_lane, -1.200, 0.500)
                    metrics['nom_sp'] = nom_sp
                    metrics['nom_ep'] = nom_ep

                    # Compute actual physical start/end points using MoCap trajectory at wp_events timestamps
                    def _get_mocap_coords(df, t):
                        if df.empty or t is None or np.isnan(t):
                            return (None, None, None)
                        idx = (df['t'] - t).abs().idxmin()
                        return (float(df.loc[idx, 'x']), float(df.loc[idx, 'y']), float(df.loc[idx, 'z']))

                    metrics['act_sp'] = _get_mocap_coords(df_mocap, wp_events.get('WP2') or wp_events.get('WP1'))
                    metrics['act_ep'] = _get_mocap_coords(df_mocap, wp_events.get('WP3'))

                    # Attach declared sweep speed to metrics
                    metrics['sweep_speed'] = sweep_speed

                    # Battery % at Exp. Start-point (WP1 / WP2, whichever is earliest available)
                    t_exp_start = wp_events.get('WP2')
                    if t_exp_start is not None and not df_bat.empty:
                        from dev_logs.analysis.kinematics.kin_calculator import query_battery
                        bat_pct_str, _ = query_battery(df_bat, t_exp_start)
                        try:
                            metrics['battery_at_start'] = float(bat_pct_str.replace('%', ''))
                        except (ValueError, AttributeError):
                            metrics['battery_at_start'] = None
                    else:
                        metrics['battery_at_start'] = None

                    # Load raw PX4 ULog metrics if available
                    flight_dir = os.path.dirname(pass_path)
                    motor_metrics = get_ulog_motor_metrics(flight_dir, pass_path, bag_start_ns, wp_events)
                    metrics.update(motor_metrics)

                    # Per-pass active flight time (takeoff → disarming, from MCAP)
                    active_flight_time_sec = float(disarming_time - takeoff_time) if disarming_time and takeoff_time else None
                    metrics['active_flight_time_sec'] = active_flight_time_sec

                    # Battery consumption rates — flight-level, from flights_battery_efficiency
                    # Per-pass MCAP windows are ~10 s, far too short for meaningful rates.
                    # The battery efficiency table provides rates computed from the full
                    # unsliced MCAP takeoff→landing window.
                    cap_drain, v_drop = _get_battery_rates(base_folder)
                    metrics['capacity_drain_rate_pct_per_min'] = cap_drain
                    metrics['voltage_drop_rate_v_per_min'] = v_drop

                    # ── Idempotency guard: skip passes already in the database ──
                    # is_already_cached() checks two conditions:
                    #   1. A row with this pass_name exists in the database.
                    #   2. All columns listed in check_columns are present in the
                    #      table schema AND have non-NULL values for this row.
                    #
                    # Condition 2 is critical: if we add new metrics columns to
                    # the pipeline (e.g., motor_imbalance_after added in a later
                    # commit), rows inserted by a previous pipeline run will fail
                    # the column check and be re-processed. This means we never
                    # need to manually invalidate the DB after schema changes —
                    # simply re-running the pipeline fills in the new columns.
                    #
                    # The check_columns list is intentionally long: every column
                    # that has been added at any point is enumerated so that
                    # schema evolution is handled automatically.
                    if is_already_cached(pass_name, check_columns=['impact_detected', 'nom_sp_x', 'before_impact_accel', 'imu_peak_accel', 'imu_std_ay', 'motor_avg_before', 'e_sp_timestamp_PX4', 'e_impact_timestamp_PX4', 'allocator_saturation_duration_sec', 'max_unallocated_torque', 'thrust_setpoint_achieved_pct', 'roll_rate_error_rms', 'pitch_rate_error_rms', 'yaw_rate_error_rms', 'active_flight_time_sec', 'voltage_drop_rate_v_per_min', 'capacity_drain_rate_pct_per_min', 'max_actuator_output', 'path_spread_rmsld', 'imu_ax_spread_impact', 'imu_delta_v_z', 'imu_delta_v_x', 'imu_delta_v_y']):
                        print(f"[{pi}/{total_in_flight}] ⏭️  '{pass_name}' already in database, skipping insert.")
                    else:
                        insert_or_replace_flight(pass_name, condition_label, metrics)

                    # Generate and save all 5 high-fidelity plots to the individual flight capsule directory
                    flight_dir = os.path.dirname(pass_path)
                    
                    if metrics.get('impact_detected', 0) == 1 or force_plot:
                        pass_prefix = f"pass{pass_idx:02d}_"
                        
                        # 1. Trajectory Top-Down 2D Spatial Plot
                        traj_capsule_path = os.path.join(flight_dir, f"{pass_prefix}trajectory_top_down.png")
                        plot_trajectory(df_mocap, wp_events, col_x_flight, col_y_flight, 
                                        cage_diameter, column_diameter, output_path=traj_capsule_path, flight_name=pass_name,
                                        dynamic_waypoints=dynamic_waypoints, df_column=df_column,
                                        df_setpoint=df_setpoint, show_plot=False)
                        
                        # Build segment events list for IMU / battery plots
                        events_log = build_events_log(df_mocap, df_bat, arming_time, takeoff_time, disarming_time, wp_events, achieved_angle=metrics.get('achieved_impact_angle'))

                        # (MoCap-based kinetic_profile.png / kinetic_profile_raw.png retired 2026-06-10 —
                        #  EKF kinetic profile is the sole velocity/acceleration plot from the pipeline.)

                        # 2. EKF Kinetic Profile (EKF velocity + tangential accel, no MoCap rate)
                        ekf_kinetic_path = os.path.join(flight_dir, f"{pass_prefix}ekf_kinetic_profile.png")
                        if df_ekf_kin is not None and not df_ekf_kin.empty:
                            plot_ekf_kinetic_profile(
                                ekf_t=df_ekf_kin["t"].values,
                                ekf_speed=df_ekf_kin["speed"].values,
                                ekf_rate=100.0,  # target_freq in compute_ekf_kinematics()
                                df_mocap=df_mocap,
                                wp_events=wp_events,
                                arming_time=arming_time,
                                flight_name=pass_name,
                                condition=condition_label,
                                achieved_angle=metrics.get('achieved_impact_angle'),
                                output_path=ekf_kinetic_path,
                                show_plot=False,
                            )
                        else:
                            print(f"  ⚡ EKF kinetic profile: not available (no vehicle_odometry data)")

                        # (Battery voltage sag plot retired from pipeline 2026-06-10)

                        # 4. Physical IMU Dynamics Plot (Acceleration & Gyro deviation/surge)
                        imu_dyn_capsule_path = os.path.join(flight_dir, f"{pass_prefix}imu_dynamics.png")
                        plot_imu_dynamics(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log,
                                          label=condition_label, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                          output_path=imu_dyn_capsule_path, show_plot=False)
                        
                        # 5. Raw IMU XYZ Components Plot
                        imu_xyz_capsule_path = os.path.join(flight_dir, f"{pass_prefix}imu_xyz_components.png")
                        plot_imu_xyz_components(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log,
                                                label=condition_label, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                                output_path=imu_xyz_capsule_path, show_plot=False)
                        
                        # 6. Actuators and Status
                        act_capsule_path = os.path.join(flight_dir, f"{pass_prefix}actuators_profile.png")
                        ulg_files = glob.glob(os.path.join(flight_dir, "*.ulg"))
                        if ulg_files:
                            offset_sec = metrics.get('offset_sec', 0.0)
                            plot_actuators_and_status(ulg_files[0], offset_sec, bag_start_ns, wp_events, arming_time, pass_name, condition_label, act_capsule_path, show_plot=False)
                            
                            # 7. Control Allocator Saturation
                            sat_capsule_path = os.path.join(flight_dir, f"{pass_prefix}control_allocator_saturation.png")
                            plot_control_allocator_saturation(ulg_files[0], offset_sec, bag_start_ns, wp_events, pass_name, condition_label, sat_capsule_path, show_plot=False)
                            
                            # 8. PID Rate Tracking
                            pid_capsule_path = os.path.join(flight_dir, f"{pass_prefix}pid_rate_tracking.png")
                            plot_pid_rate_tracking(ulg_files[0], offset_sec, bag_start_ns, wp_events, pass_name, condition_label, pid_capsule_path, show_plot=False)
                    else:
                        print(f"[{pi}/{total_in_flight}] ⏭️  No collision detected in {pass_name}, skipping plot generation.")

                    metrics_list.append(metrics)
                    details_list.append({
                        'flight_name': pass_name,
                        'df_mocap': df_mocap,
                        'df_mocap_raw': df_mocap_raw,
                        'df_bat': df_bat,
                        'df_imu': df_imu,
                        'df_column': df_column,
                        'df_setpoint': df_setpoint,
                        'col_x_flight': col_x_flight,
                        'col_y_flight': col_y_flight,
                        'arming_time': arming_time,
                        'takeoff_time': takeoff_time,
                        'disarming_time': disarming_time,
                        'wp_events': wp_events,
                        'achieved_impact_angle': metrics['achieved_impact_angle'],
                        'mocap_rate': mocap_rate,
                        'dynamic_waypoints': dynamic_waypoints
                    })
            except Exception as e:
                print(f"[ERROR] Failed processing flight {f_name}: {e}")
                import traceback; traceback.print_exc()

        return metrics_list, details_list

    force_plot = kwargs.get('force_plot', False)

    # Process all flights
    print("⏳ Processing 'Rotating Cage' passes...")
    metrics_rotating, details_rotating = process_flights(flights_rotating_cage, "Rotating Cage", force_plot=force_plot)
    print(f"✅ Successfully processed {len(metrics_rotating)} passes.\n")

    print("⏳ Processing 'Fixed Cage' passes...")
    metrics_fixed, details_fixed = process_flights(flights_fixed_cage, "Fixed Cage", force_plot=force_plot)
    print(f"✅ Successfully processed {len(metrics_fixed)} passes.\n")

    # Render representative rotating cage flight
    if details_rotating:
        rep_idx = min(max(0, representative_rotating_cage), len(details_rotating) - 1)
        rep = details_rotating[rep_idx]
        rep_metrics = metrics_rotating[rep_idx]
        
        print(f"📈 --- Representative Pass: ROTATING CAGE ({rep['flight_name']}) ---")
        if rep_metrics.get('achieved_impact_angle') is not None:
            print(f"💥 Achieved Contact Impact Angle: {rep_metrics['achieved_impact_angle']:.1f}° (Nominal Target: {angle_deg}°)")
        
        # Build and display events table
        events = build_events_log(rep['df_mocap'], rep['df_bat'], rep['arming_time'], 
                                  rep['takeoff_time'], rep['disarming_time'], rep['wp_events'],
                                  achieved_angle=rep.get('achieved_impact_angle'))
        df_events_table = pd.DataFrame(events)
        
        print("\n📊 CHRONOLOGICAL FLIGHT EVENTS & BATTERY AUDIT JOURNAL (ROTATING CAGE)")
        display(HTML(df_events_table.to_html(index=False)))
        
        if rep_metrics.get('achieved_impact_angle') is not None:
            impact_str = f"💥 Column Impact (Achieved Angle: {rep_metrics['achieved_impact_angle']:.1f}°)"
            if rep_metrics.get('impact_speed') is not None:
                impact_str += f" - Speed at impact: {rep_metrics['impact_speed']:.3f} m/s"
            if rep_metrics.get('impact_accel') is not None:
                impact_str += f" - Acceleration at impact: {rep_metrics['impact_accel']:.3f} m/s²"
            print(impact_str)
            
        print("-" * 80)
        
        # Render trajectory plot using the flight's exact dynamic column coordinates
        clean_label = label.lower().replace(' ', '_').replace('°', 'deg')
        output_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"trajectory_{clean_label}_rotating_cage.png")
        plot_trajectory(rep['df_mocap'], rep['wp_events'], rep['col_x_flight'], rep['col_y_flight'], 
                        cage_diameter, column_diameter, output_path=output_plot_path, flight_name=rep['flight_name'],
                        dynamic_waypoints=rep['dynamic_waypoints'], df_column=rep['df_column'],
                        df_setpoint=rep.get('df_setpoint'))
        
        # Render raw velocity profile
        plot_velocity_profile(rep['df_mocap_raw'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Raw MoCap", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'), mocap_rate=rep['mocap_rate'], condition="Rotating Cage (Raw)",
                              is_raw=True)
        
        # Render splined velocity profile
        plot_velocity_profile(rep['df_mocap'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Splined MoCap", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'), mocap_rate=rep['mocap_rate'], condition="Rotating Cage (Splined)",
                              df_raw=rep['df_mocap_raw'], is_raw=False)
        
        # Render battery profile
        plot_battery_sag(rep['df_bat'], rep['takeoff_time'], rep['wp_events'], rep['arming_time'], label="Rotating Cage", flight_name=rep['flight_name'],
                         achieved_angle=rep_metrics.get('achieved_impact_angle'))
 
        # Render physical IMU Dynamics & raw components (XYZ-RGB standard)
        plot_imu_dynamics(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                          rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage", flight_name=rep['flight_name'],
                          achieved_angle=rep_metrics.get('achieved_impact_angle'))
        
        plot_imu_xyz_components(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                                rep['takeoff_time'], rep['disarming_time'], events, label="Rotating Cage", flight_name=rep['flight_name'],
                                achieved_angle=rep_metrics.get('achieved_impact_angle'))
        
        # Render Actuators, Control Allocator, and PID tracking plots for Rotating Cage representative flight
        rep_flight_dir = os.path.join(project_root, rep['flight_name'].split(" - Pass")[0])
        rep_ulg_files = glob.glob(os.path.join(rep_flight_dir, "*.ulg"))
        if rep_ulg_files:
            offset_sec = rep_metrics.get('offset_sec', 0.0)
            act_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"actuators_{clean_label}_rotating_cage.png")
            plot_actuators_and_status(rep_ulg_files[0], offset_sec, bag_start_ns, rep['wp_events'], rep['arming_time'], rep['flight_name'], "Rotating Cage", act_plot_path)
            
            sat_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"allocator_{clean_label}_rotating_cage.png")
            plot_control_allocator_saturation(rep_ulg_files[0], offset_sec, bag_start_ns, rep['wp_events'], rep['flight_name'], "Rotating Cage", sat_plot_path)
            
            pid_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"pid_{clean_label}_rotating_cage.png")
            plot_pid_rate_tracking(rep_ulg_files[0], offset_sec, bag_start_ns, rep['wp_events'], rep['flight_name'], "Rotating Cage", pid_plot_path)
            
        print("\n" + "="*80 + "\n")
  
    # Render representative fixed cage flight
    if details_fixed:
        rep_idx = min(max(0, representative_fixed_cage), len(details_fixed) - 1)
        rep = details_fixed[rep_idx]
        rep_metrics = metrics_fixed[rep_idx]
        
        print(f"📈 --- Representative Pass: FIXED CAGE ({rep['flight_name']}) ---")
        if rep_metrics.get('achieved_impact_angle') is not None:
            print(f"💥 Achieved Contact Impact Angle: {rep_metrics['achieved_impact_angle']:.1f}° (Nominal Target: {angle_deg}°)")
        
        # Build and display events table
        events = build_events_log(rep['df_mocap'], rep['df_bat'], rep['arming_time'], 
                                  rep['takeoff_time'], rep['disarming_time'], rep['wp_events'],
                                  achieved_angle=rep.get('achieved_impact_angle'))
        df_events_table = pd.DataFrame(events)
        
        print("\n📊 CHRONOLOGICAL FLIGHT EVENTS & BATTERY AUDIT JOURNAL (FIXED CAGE)")
        display(HTML(df_events_table.to_html(index=False)))
        
        if rep_metrics.get('achieved_impact_angle') is not None:
            impact_str = f"💥 Column Impact (Achieved Angle: {rep_metrics['achieved_impact_angle']:.1f}°)"
            if rep_metrics.get('impact_speed') is not None:
                impact_str += f" - Speed at impact: {rep_metrics['impact_speed']:.3f} m/s"
            if rep_metrics.get('impact_accel') is not None:
                impact_str += f" - Acceleration at impact: {rep_metrics['impact_accel']:.3f} m/s²"
            print(impact_str)
            
        print("-" * 80)
        
        # Render trajectory plot using the flight's exact dynamic column coordinates
        clean_label = label.lower().replace(' ', '_').replace('°', 'deg')
        output_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"trajectory_{clean_label}_fixed_cage.png")
        plot_trajectory(rep['df_mocap'], rep['wp_events'], rep['col_x_flight'], rep['col_y_flight'], 
                        cage_diameter, column_diameter, output_path=output_plot_path, flight_name=rep['flight_name'],
                        dynamic_waypoints=rep['dynamic_waypoints'], df_column=rep['df_column'],
                        df_setpoint=rep.get('df_setpoint'))
        
        # Render raw velocity profile
        plot_velocity_profile(rep['df_mocap_raw'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Raw MoCap", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'), mocap_rate=rep['mocap_rate'], condition="Fixed Cage (Raw)",
                              is_raw=True)
        
        # Render splined velocity profile
        plot_velocity_profile(rep['df_mocap'], rep['wp_events'], rep['arming_time'], 
                              rep['takeoff_time'], rep['disarming_time'], events, label="Splined MoCap", flight_name=rep['flight_name'],
                              achieved_angle=rep_metrics.get('achieved_impact_angle'), mocap_rate=rep['mocap_rate'], condition="Fixed Cage (Splined)",
                              df_raw=rep['df_mocap_raw'], is_raw=False)
        
        # Render battery profile
        plot_battery_sag(rep['df_bat'], rep['takeoff_time'], rep['wp_events'], rep['arming_time'], label="Fixed Cage", flight_name=rep['flight_name'],
                         achieved_angle=rep_metrics.get('achieved_impact_angle'))
 
        # Render physical IMU Dynamics & raw components (XYZ-RGB standard)
        plot_imu_dynamics(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                          rep['takeoff_time'], rep['disarming_time'], events, label="Fixed Cage", flight_name=rep['flight_name'],
                          achieved_angle=rep_metrics.get('achieved_impact_angle'))
        
        plot_imu_xyz_components(rep['df_imu'], rep['wp_events'], rep['arming_time'],
                                rep['takeoff_time'], rep['disarming_time'], events, label="Fixed Cage", flight_name=rep['flight_name'],
                                achieved_angle=rep_metrics.get('achieved_impact_angle'))
        
        # Render Actuators, Control Allocator, and PID tracking plots for Fixed Cage representative flight
        rep_flight_dir = os.path.join(project_root, rep['flight_name'].split(" - Pass")[0])
        rep_ulg_files = glob.glob(os.path.join(rep_flight_dir, "*.ulg"))
        if rep_ulg_files:
            offset_sec = rep_metrics.get('offset_sec', 0.0)
            act_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"actuators_{clean_label}_fixed_cage.png")
            plot_actuators_and_status(rep_ulg_files[0], offset_sec, bag_start_ns, rep['wp_events'], rep['arming_time'], rep['flight_name'], "Fixed Cage", act_plot_path)
            
            sat_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"allocator_{clean_label}_fixed_cage.png")
            plot_control_allocator_saturation(rep_ulg_files[0], offset_sec, bag_start_ns, rep['wp_events'], rep['flight_name'], "Fixed Cage", sat_plot_path)
            
            pid_plot_path = os.path.join(project_root, "dev_logs", "analysis", "graphics", f"pid_{clean_label}_fixed_cage.png")
            plot_pid_rate_tracking(rep_ulg_files[0], offset_sec, bag_start_ns, rep['wp_events'], rep['flight_name'], "Fixed Cage", pid_plot_path)
        print("\n" + "="*80 + "\n")

    # Render side-by-side comparative boxplots
    if metrics_rotating or metrics_fixed:
        plot_angle_boxplots(metrics_rotating, metrics_fixed, label=label)

    # Output consolidated thesis experiments summary from SQLite database
    print("\n" + "="*80)
    print("📚 CONSOLIDATED MASTER THESIS COLLISION EXPERIMENTS DATABASE SUMMARY")
    print("="*80)
    print(get_database_summary_markdown())
    print("="*80 + "\n")

    return {
        'label': label,
        'angle_deg': angle_deg,
        'metrics_rotating_cage': metrics_rotating,
        'metrics_fixed_cage': metrics_fixed
    }


# ─────────────────────────────────────────────────────────────────────────────
# Standalone DB population mode
# Run as:  python3 -m dev_logs.analysis.database.db_pipeline
#
# Scans dev_logs/flights/ for *-passXX.mcap files (only approved flights),
# runs analysis on each, and populates / updates the SQLite database.
# Already-cached passes are skipped (idempotent).
#
# Flight filtering by cutoff:
#   The APPROVED_CUTOFF timestamp (from db_manager SSoT) defines the earliest
#   flight that has been manually reviewed and approved for inclusion. Flights
#   recorded before this date are skipped — they may be test flights, hardware
#   debugging sessions, or flights with known sensor failures.
#
#   The --today flag overrides the cutoff to today's date, which is useful
#   during an experimental session: run the pipeline immediately after each
#   flight to get real-time feedback without re-processing the entire history.
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import re as _re
    import glob as _glob
    import argparse
    import datetime

    # Dynamically resolve today's date formatted as YYYYMMDD-0000.
    # The format matches the flight folder naming convention
    # (e.g., "20260613-0001 Rotating Cage 45deg"), so the cutoff is a
    # lexicographic string comparison against folder names.
    _today_cutoff = datetime.date.today().strftime("%Y%m%d-0000")

    parser = argparse.ArgumentParser(description="Standalone DB population pipeline")
    parser.add_argument('--cutoff', '-c', type=str, default=APPROVED_CUTOFF,
                        help="Skipping any flight directories recorded before this timestamp in YYYYMMDD-HHMM format (default from SSoT: {})".format(APPROVED_CUTOFF))
    parser.add_argument('--today', '-t', action='store_true',
                        help=f"Shortcut to process only today's flights (sets cutoff to {_today_cutoff})")
    parser.add_argument('--force-plot', '-f', action='store_true',
                        help="Force plot generation even for passes without detected collisions")

    args = parser.parse_args()

    # Override the SSoT cutoff when --today is passed. This is a local
    # override for the current run only — it does not modify the SSoT file.
    APPROVED_CUTOFF = _today_cutoff if args.today else args.cutoff

    _current_dir = os.path.dirname(os.path.abspath(__file__))
    _project_root = os.path.abspath(os.path.join(_current_dir, "..", "..", ".."))
    _flights_dir  = os.path.join(_project_root, "dev_logs", "flights")

    drone_tracker_name, system_config = load_drone_metadata(_project_root)
    primary_body  = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "primary"),  {})
    obstacle_body = next((b for b in system_config.get("tracked_bodies", []) if b.get("role") == "obstacle"), {})
    cage_diameter   = primary_body.get("cage_diameter_m", 0.358)
    column_diameter = obstacle_body.get("diameter_m", 0.09)
    cage_radius   = cage_diameter / 2.0
    column_radius = column_diameter / 2.0

    # ── Condition inference from folder name ────────────────────────────────
    # Flight folder names follow the convention:
    #   "YYYYMMDD-NNNN Rotating Cage XXdeg"
    #   "YYYYMMDD-NNNN Fixed Cage XXdeg"
    # We use simple substring matching against the lowercased folder name.
    # Anything not containing "rotating" defaults to "Fixed Cage", which is
    # safe because the experiment has exactly two cage conditions.
    def _infer_condition(folder_name):
        name = folder_name.lower()
        if "rotating" in name:
            return "Rotating Cage"
        return "Fixed Cage"

    # ── Collect all approved pass files ──────────────────────────────────────
    # is_approved_flight() enforces the cutoff timestamp and any explicit
    # inclusion/exclusion rules from the SSoT db_manager configuration.
    # Only flight folders that pass this gate are considered for processing.
    # Each flight folder may contain multiple pass MCAPs (e.g., pass01..pass08).
    all_pass_files = []
    for folder in sorted(os.listdir(_flights_dir)):
        if not is_approved_flight(folder):
            continue
        folder_path = os.path.join(_flights_dir, folder)
        passes = sorted(_glob.glob(os.path.join(folder_path, "*-pass*.mcap")))
        for p in passes:
            all_pass_files.append((folder, folder_path, p))

    if not all_pass_files:
        print("⚠️  No pass files found. Run mcap_segmenter.py first.")
        sys.exit(0)

    print(f"\n🗄️  DB Population Mode — found {len(all_pass_files)} pass file(s) across approved flights.\n")

    try:
        sys.path.insert(0, _project_root)
        from drone_control.missions.exp_collision_75deg import ExpCollision75Deg as _M
        sweep_speed_declared = _M().sweep_speed
    except Exception:
        sweep_speed_declared = None

    populated, skipped, errors = 0, 0, 0

    global_battery_cache = {}
    def _get_global_battery(flight_folder_path):
        import glob as _glob
        import os
        ulg_files = _glob.glob(os.path.join(flight_folder_path, "*.ulg"))
        if not ulg_files:
            return None, None, None
        try:
            import pyulog
            ulog = pyulog.ULog(ulg_files[0], message_name_filter_list=['vehicle_status', 'battery_status'])
            
            ds_bat = ulog.get_dataset('battery_status')
            t_bat_us = ds_bat.data['timestamp']
            voltage = ds_bat.data['voltage_v']
            remaining = ds_bat.data['remaining']
            
            ds_stat = ulog.get_dataset('vehicle_status')
            t_stat_us = ds_stat.data['timestamp']
            arming_state = ds_stat.data['arming_state']
            
            arming_time_us = None
            disarming_time_us = None
            for i in range(len(t_stat_us)):
                if arming_state[i] == 2 and arming_time_us is None:
                    arming_time_us = t_stat_us[i]
                elif arming_state[i] == 1 and arming_time_us is not None and disarming_time_us is None:
                    disarming_time_us = t_stat_us[i]
            
            if arming_time_us is not None:
                if disarming_time_us is None:
                    disarming_time_us = t_bat_us[-1]
                    
                bat_mask = (t_bat_us >= arming_time_us) & (t_bat_us <= disarming_time_us)
                if bat_mask.any():
                    v_start = voltage[bat_mask][0]
                    v_end = voltage[bat_mask][-1]
                    c_start = remaining[bat_mask][0]
                    c_end = remaining[bat_mask][-1]
                    flight_time_sec = (disarming_time_us - arming_time_us) / 1e6
                    if flight_time_sec > 0:
                        flight_time_min = flight_time_sec / 60.0
                        return float(flight_time_sec), float((v_start - v_end) / flight_time_min), float((c_start - c_end) * 100.0 / flight_time_min)
        except Exception as e:
            print(f"  [WARN] Failed to parse global battery metrics: {e}")
        return None, None, None
    # ── Per-pass processing loop ────────────────────────────────────────────
    # Each iteration processes exactly one pass MCAP file, computes all
    # metrics (MoCap kinematics, waypoint detection, EKF kinematics, ULog
    # motor/allocator/PID metrics, battery rates), and inserts a single row
    # into the SQLite database.
    #
    # The pass name is constructed as "<flight_folder> - Pass-NN" which is
    # the primary key in the database — no two passes share the same key.
    total_passes = len(all_pass_files)
    for i, (folder_name, folder_path, pass_path) in enumerate(all_pass_files, 1):
        pass_basename = os.path.basename(pass_path)
        # Parse pass index from filename, e.g., "...-pass03.mcap" → 3
        m = _re.search(r'-pass(\d+)\.mcap$', pass_basename)
        if not m:
            continue
        pass_idx  = int(m.group(1))
        pass_name = f"{folder_name} - Pass-{pass_idx:02d}"

        # ── Exclusion gate ──
        if is_excluded_pass(pass_name):
            from dev_logs.analysis.database.db_manager import get_exclusion_info
            _, reason, _ = get_exclusion_info(pass_name)
            print(f"[{i}/{total_passes}] ⏭️  Skipping (excluded: {reason}): {pass_name}")
            skipped += 1
            continue

        if False:  # Temporarily bypassed to populate new motor metrics columns for all flights
            print(f"[{i}/{total_passes}] ⏭️  Skipping (already cached): {pass_name}")
            skipped += 1
            continue

        print(f"\n[{i}/{total_passes}] 🔄 Processing: {pass_name}")
        try:
            # Load the self-contained pass MCAP
            from mcap_ros2.reader import read_ros2_messages as _read_msgs
            topic_data = {}
            for msg in _read_msgs(pass_path):
                t = msg.channel.topic
                if t not in topic_data:
                    topic_data[t] = []
                topic_data[t].append(msg)

            if not topic_data:
                print(f"   ⚠️  Empty MCAP, skipping.")
                errors += 1
                continue

            bag_start_ns = min(m.log_time_ns for msgs in topic_data.values() for m in msgs)
            dfs = build_dataframes(topic_data, drone_tracker_name, bag_start_ns)

            df_mocap  = dfs['mocap']
            df_setpoint = dfs.get('setpoint', pd.DataFrame())
            df_bat    = dfs['battery']
            df_imu    = dfs.get('imu', pd.DataFrame())
            df_column = dfs.get('column', pd.DataFrame())
            df_odom   = dfs.get('odom', pd.DataFrame())
            arming_time       = dfs['arming_time']
            disarming_time    = dfs.get('disarming_time')
            if disarming_time is None:
                disarming_time = arming_time + 10.0
            mocap_rate        = dfs.get('mocap_rate', 240.0)
            dynamic_waypoints = dfs.get('dynamic_waypoints', [])

            if df_mocap.empty:
                print(f"   ⚠️  No MoCap data, skipping.")
                errors += 1
                continue

            df_mocap_raw = compute_mocap_kinematics(df_mocap.copy(), resample=False)
            df_mocap = compute_mocap_kinematics(df_mocap, resample=True)

            # EKF kinematics (same as main pipeline above)
            df_ekf_kin = compute_ekf_kinematics(df_odom, df_mocap)

            takeoff_mask = df_mocap['z'] > 0.15
            takeoff_time = df_mocap.loc[takeoff_mask, 't'].iloc[0] if takeoff_mask.any() else arming_time + 2.0

            col_x, col_y = (df_column['x'].head(50).mean(), df_column['y'].head(50).mean()) \
                if (df_column is not None and not df_column.empty) else (0.408, 0.358)

            wp_events_list = find_waypoint_events(
                df_mocap, df_setpoint, takeoff_time, label=pass_name,
                column_x=col_x, column_y=col_y,
                column_radius=column_radius, cage_radius=cage_radius,
                return_all=True, dynamic_waypoints=dynamic_waypoints
            )

            if not wp_events_list:
                print(f"   ⚠️  No waypoint events found, skipping.")
                errors += 1
                continue

            wp_events = wp_events_list[0]  # Pass file contains exactly one pass

            metrics = compute_flight_metrics(df_mocap, wp_events, col_x, col_y, column_radius, cage_radius, df_imu=df_imu, df_ekf_kin=df_ekf_kin)
            closest_clearance = metrics.get('closest_clearance')
            metrics['impact_detected'] = 1 if (closest_clearance is not None and closest_clearance < 0.0) else 0

            # Read declared nominal waypoints from mission SSoT
            _m = None
            try:
                from dev_logs.analysis.kinematics.kin_calculator import detect_mission_class
                _m = detect_mission_class(df_setpoint, label=pass_name)
                nom_sp = (_m.exp_sp[0], _m.exp_sp[1], _m.exp_sp[2])
                nom_ep = (_m.exp_ep[0], _m.exp_ep[1], _m.exp_ep[2])
            except Exception:
                is_45 = pass_name and "45" in pass_name.lower()
                x_lane = 0.248 if is_45 else 0.186
                nom_sp = (x_lane, 0.950, 0.500)
                nom_ep = (x_lane, -1.200, 0.500)
            metrics['nom_sp'] = nom_sp
            metrics['nom_ep'] = nom_ep

            # Compute actual physical start/end points using MoCap trajectory at wp_events timestamps
            def _get_mocap_coords(df, t):
                if df.empty or t is None or np.isnan(t):
                    return (None, None, None)
                idx = (df['t'] - t).abs().idxmin()
                return (float(df.loc[idx, 'x']), float(df.loc[idx, 'y']), float(df.loc[idx, 'z']))

            metrics['act_sp'] = _get_mocap_coords(df_mocap, wp_events.get('WP2') or wp_events.get('WP1'))
            metrics['act_ep'] = _get_mocap_coords(df_mocap, wp_events.get('WP3'))

            metrics['sweep_speed'] = _m.sweep_speed if _m is not None else sweep_speed_declared

            # Battery % at Exp. Start-point
            t_exp_start = wp_events.get('WP2')
            if t_exp_start is not None and not df_bat.empty:
                from dev_logs.analysis.kinematics.kin_calculator import query_battery as _qb
                bat_pct_str, _ = _qb(df_bat, t_exp_start)
                try:
                    metrics['battery_at_start'] = float(bat_pct_str.replace('%', ''))
                except (ValueError, AttributeError):
                    metrics['battery_at_start'] = None
            else:
                metrics['battery_at_start'] = None

            # Load raw PX4 ULog metrics if available
            motor_metrics = get_ulog_motor_metrics(folder_path, pass_path, bag_start_ns, wp_events)
            metrics.update(motor_metrics)

            # Per-pass active flight time (takeoff → disarming, from MCAP)
            active_flight_time_sec = float(disarming_time - takeoff_time) if disarming_time and takeoff_time else None
            metrics['active_flight_time_sec'] = active_flight_time_sec

            # Battery consumption rates — flight-level, from flights_battery_efficiency
            cap_drain, v_drop = _get_battery_rates(folder_name)
            metrics['capacity_drain_rate_pct_per_min'] = cap_drain
            metrics['voltage_drop_rate_v_per_min'] = v_drop

            condition = _infer_condition(folder_name)
            
            # Skip if already cached
            if is_already_cached(pass_name, check_columns=['impact_detected', 'nom_sp_x', 'before_impact_accel', 'imu_peak_accel', 'imu_std_ay', 'motor_avg_before', 'e_sp_timestamp_PX4', 'e_impact_timestamp_PX4', 'allocator_saturation_duration_sec', 'max_unallocated_torque', 'thrust_setpoint_achieved_pct', 'roll_rate_error_rms', 'pitch_rate_error_rms', 'yaw_rate_error_rms', 'active_flight_time_sec', 'voltage_drop_rate_v_per_min', 'capacity_drain_rate_pct_per_min', 'max_actuator_output', 'path_spread_rmsld', 'imu_ax_spread_impact', 'imu_delta_v_z', 'imu_delta_v_x', 'imu_delta_v_y']):
                print(f"[{i}/{total_passes}] ⏭️  '{pass_name}' already in database, skipping insert.")
            else:
                insert_or_replace_flight(pass_name, condition, metrics)

            # Generate and save high-fidelity plots to the individual flight capsule directory
            if metrics.get('impact_detected', 0) == 1 or args.force_plot:
                pass_prefix = f"pass{pass_idx:02d}_"
                
                # 1. Trajectory Top-Down 2D Spatial Plot
                traj_capsule_path = os.path.join(folder_path, f"{pass_prefix}trajectory_top_down.png")
                plot_trajectory(df_mocap, wp_events, col_x, col_y, 
                                cage_diameter, column_diameter, output_path=traj_capsule_path, flight_name=pass_name,
                                dynamic_waypoints=dynamic_waypoints, df_column=df_column,
                                df_setpoint=df_setpoint, show_plot=False)
                
                # Build segment events list for IMU / battery plots
                events_log = build_events_log(df_mocap, df_bat, arming_time, takeoff_time, disarming_time, wp_events, achieved_angle=metrics.get('achieved_impact_angle'))

                # (MoCap-based kinetic_profile.png / kinetic_profile_raw.png retired 2026-06-10)

                # 2. EKF Kinetic Profile (EKF velocity + tangential accel, no MoCap rate)
                ekf_kinetic_path = os.path.join(folder_path, f"{pass_prefix}ekf_kinetic_profile.png")
                if df_ekf_kin is not None and not df_ekf_kin.empty:
                    plot_ekf_kinetic_profile(
                        ekf_t=df_ekf_kin["t"].values,
                        ekf_speed=df_ekf_kin["speed"].values,
                        ekf_rate=100.0,
                        df_mocap=df_mocap,
                        wp_events=wp_events,
                        arming_time=arming_time,
                        flight_name=pass_name,
                        condition=condition,
                        achieved_angle=metrics.get('achieved_impact_angle'),
                        output_path=ekf_kinetic_path,
                        show_plot=False,
                    )
                else:
                    print(f"  ⚡ EKF kinetic profile: not available (no vehicle_odometry data)")

                # (Battery voltage sag plot retired from pipeline 2026-06-10)

                # 4. Physical IMU Dynamics Plot (Acceleration & Gyro deviation/surge)
                imu_dyn_capsule_path = os.path.join(folder_path, f"{pass_prefix}imu_dynamics.png")
                plot_imu_dynamics(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log,
                                  label=condition, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                  output_path=imu_dyn_capsule_path, show_plot=False)
                
                # 5. Raw IMU XYZ Components Plot
                imu_xyz_capsule_path = os.path.join(folder_path, f"{pass_prefix}imu_xyz_components.png")
                plot_imu_xyz_components(df_imu, wp_events, arming_time, takeoff_time, disarming_time, events_log,
                                        label=condition, flight_name=pass_name, achieved_angle=metrics.get('achieved_impact_angle'),
                                        output_path=imu_xyz_capsule_path, show_plot=False)

                # 6. Actuators and Status
                ulg_files = glob.glob(os.path.join(folder_path, "*.ulg"))
                if ulg_files:
                    offset_sec = metrics.get('offset_sec', 0.0)
                    act_capsule_path = os.path.join(folder_path, f"{pass_prefix}actuators_profile.png")
                    plot_actuators_and_status(ulg_files[0], offset_sec, bag_start_ns, wp_events, arming_time, pass_name, condition, act_capsule_path, show_plot=False)

                    # 7. Control Allocator Saturation
                    sat_capsule_path = os.path.join(folder_path, f"{pass_prefix}control_allocator_saturation.png")
                    plot_control_allocator_saturation(ulg_files[0], offset_sec, bag_start_ns, wp_events, pass_name, condition, sat_capsule_path, show_plot=False)

                    # 8. PID Rate Tracking
                    pid_capsule_path = os.path.join(folder_path, f"{pass_prefix}pid_rate_tracking.png")
                    plot_pid_rate_tracking(ulg_files[0], offset_sec, bag_start_ns, wp_events, pass_name, condition, pid_capsule_path, show_plot=False)
            else:
                print(f"   [{i}/{total_passes}] ⏭️  No collision detected in {pass_name}, skipping plot generation.")

            populated += 1

        except Exception as e:
            print(f"   ❌ Error: {e}")
            import traceback; traceback.print_exc()
            errors += 1

    print(f"\n{'='*80}")
    print(f"✅ DB population complete: {populated} inserted, {skipped} skipped, {errors} errors.")
    print(f"{'='*80}\n")
    print(get_database_summary_markdown())

