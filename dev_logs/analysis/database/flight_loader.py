"""Standalone flight data loader for interactive notebook use.

Provides `load_flight_data()` — a single function that loads one flight pass's
MCAP, computes kinematics (same functions as the database pipeline), and returns
everything needed for any plot type.  Also provides thin plot-wrapper functions
so notebook cells stay at 1–2 lines each.

Usage (notebook cell 0):
    from dev_logs.analysis.database.flight_loader import (
        load_flight_data,
        plot_trajectory_from, plot_velocity_profile_from, plot_battery_sag_from,
        plot_imu_dynamics_from, plot_imu_xyz_from,
        plot_actuators_from, plot_allocator_from, plot_pid_tracking_from,
        plot_ekf_velocity_from, plot_ekf_dual_comparison,
    )
    rot = load_flight_data("flight_...rotating_cage", "Pass-05")
    fix = load_flight_data("flight_...fixed_cage",   "Pass-03")

    # Then in any cell:
    plot_trajectory_from(rot)       # 2D top-down trajectory
    plot_velocity_profile_from(rot) # 3-panel kinetic profile (splined)
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

from mcap_ros2.reader import read_ros2_messages

from dev_logs.analysis.database.db_loader import (
    load_mcap,
    build_dataframes,
    load_drone_metadata,
)
from dev_logs.analysis.kinematics.kin_calculator import (
    compute_velocity,
    find_waypoint_events,
    build_events_log,
)
from dev_logs.analysis.kinematics.kin_plot_trajectory import plot_trajectory, plot_full_loop_geometry
from dev_logs.analysis.kinematics.kin_plot_kinematics import (
    plot_velocity_profile,
    plot_battery_sag,
    plot_imu_dynamics,
    plot_imu_xyz_components,
    draw_timeline_markers,
    get_timeline_limits,
    C_MOCAP,
)
from dev_logs.analysis.kinematics.kin_plot_actuators import (
    plot_actuators_and_status,
    plot_control_allocator_saturation,
    plot_pid_rate_tracking,
)


# ═══════════════════════════════════════════════════════════════════════════════
#  Core loader
# ═══════════════════════════════════════════════════════════════════════════════

def load_flight_data(flight_folder, pass_name, project_root=None,
                     window=19, polyorder=3):
    """Load one flight pass and compute kinematics.

    Returns a dict with all data needed by every plot function in the project.
    The exact same pipeline functions are used — no duplicated logic.

    Parameters
    ----------
    flight_folder : str
        Folder name under dev_logs/flights/, e.g.
        ``"flight_20260601-1756_45°_column_collision_loop_rotating_cage"``
    pass_name : str
        Pass label, e.g. ``"Pass-05"``.
    project_root : str or None
        Auto-resolved from this file's location if None.
    window, polyorder : int
        Savitzky-Golay parameters passed to :func:`compute_velocity`.

    Returns
    -------
    dict
        Keys: flight_name, condition, df_mocap, df_mocap_raw, df_bat,
        df_imu, df_setpoint, df_column, df_odom, dfs, bag_start_ns,
        arming_time, takeoff_time, disarming_time, wp_events, events_log,
        col_x, col_y, mocap_rate, dynamic_waypoints, achieved_angle,
        ulg_path, offset_sec, cage_diameter, column_diameter.
    """
    if project_root is None:
        project_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "..")
        )

    flights_dir = os.path.join(project_root, "dev_logs", "flights")
    pass_num = "".join(filter(str.isdigit, pass_name))
    mcap_path = os.path.join(
        flights_dir, flight_folder,
        f"{flight_folder}_0-pass{pass_num}.mcap"
    )

    if not os.path.exists(mcap_path):
        raise FileNotFoundError(f"MCAP not found: {mcap_path}")

    print(f"⏳ Loading {pass_name} …")
    topic_data, bag_start_ns = load_mcap(mcap_path)
    tracker_name, system_config = load_drone_metadata(project_root)
    dfs = build_dataframes(topic_data, tracker_name, bag_start_ns)

    # ── Dataframes ──
    df_mocap = dfs["mocap"].copy()
    if "z" in df_mocap.columns:
        df_mocap["z"] = df_mocap["z"] - 0.150

    df_setpoint = dfs.get("setpoint", pd.DataFrame())
    df_bat      = dfs.get("battery",   pd.DataFrame())
    df_imu      = dfs.get("imu",       pd.DataFrame())
    df_column   = dfs.get("column",    pd.DataFrame())
    df_odom     = dfs.get("odom",      pd.DataFrame())

    arming_time      = dfs.get("arming_time",       df_mocap["t"].min())
    disarming_time   = dfs.get("disarming_time",    df_mocap["t"].max())
    mocap_rate       = dfs.get("mocap_rate",        240.0)
    dynamic_waypoints = dfs.get("dynamic_waypoints", [])

    # ── Velocity ──
    print(f"⚙️  Computing kinematics (window={window}, polyorder={polyorder}) …")
    df_mocap_raw     = compute_velocity(df_mocap.copy(), window=window,
                                        polyorder=polyorder, resample=False)
    df_mocap_splined = compute_velocity(df_mocap,         window=window,
                                        polyorder=polyorder, resample=True)

    # ── Column position (dynamic or fallback) ──
    if df_column is not None and not df_column.empty:
        col_x = df_column["x"].head(50).mean()
        col_y = df_column["y"].head(50).mean()
    else:
        col_x, col_y = 0.408, 0.358

    # ── Cage / column geometry from config ──
    primary_body  = next((b for b in system_config.get("tracked_bodies", [])
                          if b.get("role") == "primary"), {})
    obstacle_body = next((b for b in system_config.get("tracked_bodies", [])
                          if b.get("role") == "obstacle"), {})
    cage_diameter   = primary_body.get("cage_diameter_m", 0.358)
    column_diameter = obstacle_body.get("diameter_m", 0.09)
    cage_radius     = cage_diameter   / 2.0
    column_radius   = column_diameter / 2.0

    # ── Takeoff time ──
    takeoff_mask = df_mocap_splined["z"] > 0.15 if not df_mocap_splined.empty else pd.Series(dtype=bool)
    takeoff_time = (
        df_mocap_splined.loc[takeoff_mask, "t"].iloc[0]
        if takeoff_mask.any() else arming_time + 2.0
    )

    # ── Waypoint events ──
    flight_name = f"{flight_folder} - {pass_name}"
    wp_events = find_waypoint_events(
        df_mocap_splined, df_setpoint, takeoff_time,
        label=flight_name,
        column_x=col_x, column_y=col_y,
        column_radius=column_radius, cage_radius=cage_radius,
        return_all=False, dynamic_waypoints=dynamic_waypoints,
    )
    if not isinstance(wp_events, dict):
        wp_events = {}

    achieved_angle = wp_events.get("achieved_impact_angle")

    # ── Events log ──
    events_log = build_events_log(
        df_mocap_splined, df_bat,
        arming_time, takeoff_time, disarming_time,
        wp_events, achieved_angle=achieved_angle,
    )

    # ── Condition ──
    condition = (
        "Rotating Cage" if "rotating_cage" in flight_folder.lower()
        else "Fixed Cage"
    )

    # ── ULog file ──
    ulg_files = glob.glob(os.path.join(flights_dir, flight_folder, "*.ulg"))
    ulg_path = ulg_files[0] if ulg_files else None

    # ── Clock-sync offset (ULog ↔ MCAP) ──
    offset_sec = _get_offset_sec(mcap_path, ulg_path, dfs)

    print(f"✅ {flight_name}  ({condition}, {len(df_mocap_splined)} MoCap samples)")

    return {
        "flight_name":      flight_name,
        "condition":        condition,
        "df_mocap":         df_mocap_splined,
        "df_mocap_raw":     df_mocap_raw,
        "df_bat":           df_bat,
        "df_imu":           df_imu,
        "df_setpoint":      df_setpoint,
        "df_column":        df_column,
        "df_odom":          df_odom,
        "dfs":              dfs,
        "bag_start_ns":     bag_start_ns,
        "arming_time":      arming_time,
        "takeoff_time":     takeoff_time,
        "disarming_time":   disarming_time,
        "wp_events":        wp_events,
        "events_log":       events_log,
        "col_x":            col_x,
        "col_y":            col_y,
        "mocap_rate":       mocap_rate,
        "dynamic_waypoints": dynamic_waypoints,
        "achieved_angle":   achieved_angle,
        "ulg_path":         ulg_path,
        "offset_sec":       offset_sec,
        "cage_diameter":    cage_diameter,
        "column_diameter":  column_diameter,
    }


def _get_offset_sec(mcap_path, ulg_path, dfs):
    """Compute clock-sync offset between MCAP and ULog timestamps."""
    # Attempt 1: timesync_status in the pass MCAP itself
    try:
        for msg in read_ros2_messages(mcap_path):
            if msg.channel.topic == "/fmu/out/timesync_status":
                return -(msg.ros_msg.observed_offset * 1e-6)
    except Exception:
        pass

    # Attempt 2: timesync_status in a sibling MCAP
    try:
        flight_dir = os.path.dirname(mcap_path)
        for mf in glob.glob(os.path.join(flight_dir, "*.mcap")):
            if mf == mcap_path:
                continue
            for msg in read_ros2_messages(mf):
                if msg.channel.topic == "/fmu/out/timesync_status":
                    return -(msg.ros_msg.observed_offset * 1e-6)
    except Exception:
        pass

    return 0.0


# ═══════════════════════════════════════════════════════════════════════════════
#  Thin plot wrappers  (notebook cells stay at 1–2 lines)
# ═══════════════════════════════════════════════════════════════════════════════

def plot_trajectory_from(data, output_path=None):
    """2D top-down trajectory plot."""
    plot_trajectory(
        data["df_mocap"], data["wp_events"],
        data["col_x"], data["col_y"],
        data["cage_diameter"], data["column_diameter"],
        flight_name=data["flight_name"],
        condition=data["condition"],
        dynamic_waypoints=data["dynamic_waypoints"],
        df_column=data["df_column"],
        df_setpoint=data["df_setpoint"],
        output_path=output_path,
        show_plot=True,
    )


def plot_velocity_profile_from(data, raw=False, output_path=None):
    """3-panel kinetic profile (velocity, tangential accel, MoCap rate)."""
    if raw:
        df      = data["df_mocap_raw"]
        df_raw  = None
        label   = "Raw MoCap"
        suffix  = " (Raw)"
    else:
        df      = data["df_mocap"]
        df_raw  = data["df_mocap_raw"]
        label   = "Splined MoCap"
        suffix  = " (Splined)"

    plot_velocity_profile(
        df, data["wp_events"],
        data["arming_time"], data["takeoff_time"], data["disarming_time"],
        data["events_log"],
        label=label,
        flight_name=data["flight_name"],
        achieved_angle=data["achieved_angle"],
        mocap_rate=data["mocap_rate"],
        condition=data["condition"] + suffix,
        output_path=output_path,
        show_plot=True, df_raw=df_raw, is_raw=raw,
    )


def plot_battery_sag_from(data):
    """Battery voltage sag and capacity depletion plot."""
    plot_battery_sag(
        data["df_bat"], data["takeoff_time"],
        data["wp_events"], data["arming_time"],
        label=data["condition"],
        flight_name=data["flight_name"],
        achieved_angle=data["achieved_angle"],
        show_plot=True,
    )


def plot_imu_dynamics_from(data, output_path=None):
    """IMU collision dynamics (accel deviation + gyro surge)."""
    plot_imu_dynamics(
        data["df_imu"], data["wp_events"],
        data["arming_time"], data["takeoff_time"], data["disarming_time"],
        data["events_log"],
        label=data["condition"],
        flight_name=data["flight_name"],
        achieved_angle=data["achieved_angle"],
        output_path=output_path,
        show_plot=True,
    )


def plot_imu_xyz_from(data, output_path=None):
    """Raw IMU X/Y/Z components (RGB-standard)."""
    plot_imu_xyz_components(
        data["df_imu"], data["wp_events"],
        data["arming_time"], data["takeoff_time"], data["disarming_time"],
        data["events_log"],
        label=data["condition"],
        flight_name=data["flight_name"],
        achieved_angle=data["achieved_angle"],
        output_path=output_path,
        show_plot=True,
    )


def plot_actuators_from(data, output_path=None):
    """Actuator motor commands + outputs + vehicle status (requires ULog)."""
    if not data.get("ulg_path"):
        print(f"⚠️  No ULog file for {data['flight_name']} — skipping.")
        return
    plot_actuators_and_status(
        data["ulg_path"], data["offset_sec"], data["bag_start_ns"],
        data["wp_events"], data["arming_time"],
        data["flight_name"], data["condition"],
        output_path=output_path, show_plot=True,
    )


def plot_allocator_from(data, output_path=None):
    """Control allocator saturation analysis (requires ULog)."""
    if not data.get("ulg_path"):
        print(f"⚠️  No ULog file for {data['flight_name']} — skipping.")
        return
    plot_control_allocator_saturation(
        data["ulg_path"], data["offset_sec"], data["bag_start_ns"],
        data["wp_events"], data["flight_name"], data["condition"],
        output_path=output_path, show_plot=True,
    )


def plot_pid_tracking_from(data, output_path=None):
    """PID rate-controller tracking performance (requires ULog)."""
    if not data.get("ulg_path"):
        print(f"⚠️  No ULog file for {data['flight_name']} — skipping.")
        return
    plot_pid_rate_tracking(
        data["ulg_path"], data["offset_sec"], data["bag_start_ns"],
        data["wp_events"], data["flight_name"], data["condition"],
        output_path=output_path, show_plot=True,
    )


# ═══════════════════════════════════════════════════════════════════════════════
#  EKF velocity helpers
# ═══════════════════════════════════════════════════════════════════════════════

def compute_ekf_velocity(data):
    """Interpolate EKF odometry velocity onto MoCap time grid.

    Returns ``{t, vx, vy, vz, speed, rate}`` or ``None`` if no EKF data.
    """
    df_odom  = data["df_odom"]
    df_mocap = data["df_mocap"]

    if df_odom.empty or "vx_ekf_raw" not in df_odom.columns:
        print(f"⚠️  No EKF velocity for {data['flight_name']}")
        return None

    t_mocap = df_mocap["t"].values
    dod = (df_odom[["t", "vx_ekf_raw", "vy_ekf_raw", "vz_ekf_raw"]]
           .dropna().sort_values("t"))

    vx = np.interp(t_mocap, dod["t"], dod["vx_ekf_raw"],
                    left=np.nan, right=np.nan)
    vy = np.interp(t_mocap, dod["t"], dod["vy_ekf_raw"],
                    left=np.nan, right=np.nan)
    vz = np.interp(t_mocap, dod["t"], dod["vz_ekf_raw"],
                    left=np.nan, right=np.nan)

    # ENU alignment (same sign convention as position in db_loader.py)
    vy = -vy
    vz = -vz
    speed = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

    rmse_vx = np.sqrt(np.nanmean((df_mocap["vx"].values - vx) ** 2))
    rmse_vy = np.sqrt(np.nanmean((df_mocap["vy"].values - vy) ** 2))
    rmse_vz = np.sqrt(np.nanmean((df_mocap["vz"].values - vz) ** 2))
    print(f"📊 EKF↔MoCap velocity RMSE ({data['flight_name']}): "
          f"vx={rmse_vx:.3f}, vy={rmse_vy:.3f}, vz={rmse_vz:.3f} m/s")

    return {
        "t": t_mocap, "vx": vx, "vy": vy, "vz": vz,
        "speed": speed, "rate": data["mocap_rate"],
    }


def plot_ekf_velocity_from(data, output_path=None):
    """EKF vs MoCap velocity side-by-side for a single flight."""
    ekf = compute_ekf_velocity(data)
    if ekf is None:
        return

    df          = data["df_mocap"]
    wp_events   = data["wp_events"]
    arming_time = data["arming_time"]
    impact_time = wp_events.get("Column Impact")
    cond        = data["condition"]
    name        = data["flight_name"]

    fig, axes = plt.subplots(3, 2, figsize=(16, 10),
                             sharex="col", sharey="row")
    fig.suptitle(f"EKF vs MoCap Velocity — <{cond}>\n{name}",
                 fontsize=13, fontweight="bold", y=0.98)

    for row, (ax_l, ax_r) in enumerate(axes):
        axis  = ["vx", "vy", "vz"][row]
        ylbl  = ["VX (East) [m/s]", "VY (North) [m/s]", "VZ (Up) [m/s]"][row]

        # Fixed Y limits for side-by-side visual consistency
        ylim = (-0.4, 0.6)

        # MoCap (left)
        ax_l.plot(ekf["t"], df[axis].values, "b-", lw=0.8, alpha=0.85,
                  label="MoCap SG")
        ax_l.set_ylabel(ylbl)
        ax_l.set_ylim(ylim)
        ax_l.set_title(f"MoCap-derived {axis.upper()}", fontsize=11, color="blue")
        ax_l.grid(True, alpha=0.3)
        if impact_time is not None:
            ax_l.axvline(x=impact_time, color="red", ls="--", lw=1.2, alpha=0.7)

        # EKF (right)
        ax_r.plot(ekf["t"], ekf[axis], "green", lw=1.0, alpha=0.9, label="EKF")
        ax_r.set_ylim(ylim)
        ax_r.set_title(f"EKF {axis.upper()} (vehicle_odometry)",
                       fontsize=11, color="green")
        ax_r.grid(True, alpha=0.3)
        if impact_time is not None:
            ax_r.axvline(x=impact_time, color="red", ls="--", lw=1.2, alpha=0.7)

    axes[-1][0].set_xlabel("Time [s]")
    axes[-1][1].set_xlabel("Time [s]")

    if impact_time is not None:
        for ax in axes.flatten():
            ax.text(impact_time + 0.03, ax.get_ylim()[1] * 0.85,
                    f"Impact\nt={impact_time:.2f}s",
                    fontsize=7, color="red", verticalalignment="top",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="white",
                              alpha=0.7))

    # Timeline limits + 1.0s tick spacing (same standard as all other plots)
    t_min_crop, t_max_crop = get_timeline_limits(
        wp_events, arming_time, df["t"], is_absolute=False
    )
    for ax in axes[-1, :]:       # bottom row only (sharex propagates upward)
        ax.set_xlim(t_min_crop, t_max_crop)
        ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))

    # ── Speed magnitude subplot ──
    fig_sp, ax_sp = plt.subplots(1, 1, figsize=(14, 4))
    ax_sp.plot(ekf["t"], df["speed"].values, "b-", lw=0.8, alpha=0.7,
               label="MoCap SG Speed")
    ax_sp.plot(ekf["t"], ekf["speed"], "green", lw=1.2, alpha=0.85,
               label="EKF Speed")
    ax_sp.set_ylabel("Speed [m/s]")
    ax_sp.set_ylim(-0.05, 0.85)
    ax_sp.set_yticks(np.arange(0, 0.9, 0.2))
    ax_sp.set_xlim(t_min_crop, t_max_crop)
    ax_sp.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax_sp.set_xlabel("Time [s]")
    ax_sp.set_title(f"Speed Comparison — {cond}")
    ax_sp.legend(loc="upper right")
    ax_sp.grid(True, alpha=0.3)
    if impact_time is not None:
        ax_sp.axvline(x=impact_time, color="red", ls="--", lw=1.2, alpha=0.7)

    plt.tight_layout()
    if output_path:
        fig_sp.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"[INFO]  EKF velocity plot saved to:  {output_path}")
    plt.show()
    print(f"✅ EKF velocity done — MoCap rate ≈ {ekf['rate']} Hz")


def plot_ekf_kinetic_from(data, output_path=None):
    """EKF-based kinetic profile: velocity + tangential accel (no MoCap rate panel).

    Thin wrapper that delegates to :func:`plot_ekf_kinetic_profile` in
    ``kin_plot_kinematics.py`` — extracts arrays from the *data* dict.
    """
    ekf = compute_ekf_velocity(data)
    if ekf is None:
        return

    from dev_logs.analysis.kinematics.kin_plot_kinematics import (
        plot_ekf_kinetic_profile,
    )
    plot_ekf_kinetic_profile(
        ekf_t=ekf["t"],
        ekf_speed=ekf["speed"],
        ekf_rate=ekf["rate"],
        df_mocap=data["df_mocap"],
        wp_events=data["wp_events"],
        arming_time=data["arming_time"],
        flight_name=data["flight_name"],
        condition=data["condition"],
        achieved_angle=data.get("achieved_angle"),
        output_path=output_path,
        show_plot=True,
    )


def plot_ekf_dual_comparison(rot_data, fix_data, output_path=None):
    """EKF vs MoCap speed for both cages in a single 2×2 figure."""
    ekf_rot = compute_ekf_velocity(rot_data)
    ekf_fix = compute_ekf_velocity(fix_data)

    if ekf_rot is None or ekf_fix is None:
        print("⚠️  Cannot plot dual EKF comparison — missing EKF data.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(18, 10))
    fig.suptitle(
        "EKF Velocity vs MoCap-Derived — Representative Flights (45°)",
        fontsize=14, fontweight="bold", y=0.98,
    )

    # ── Timeline limits for each cage (independent, per classic convention) ──
    t_min_fix, t_max_fix = get_timeline_limits(
        fix_data["wp_events"], fix_data["arming_time"],
        fix_data["df_mocap"]["t"], is_absolute=False,
    )
    t_min_rot, t_max_rot = get_timeline_limits(
        rot_data["wp_events"], rot_data["arming_time"],
        rot_data["df_mocap"]["t"], is_absolute=False,
    )

    def _flight_label(data, ax):
        """Add flight name annotation in bottom-right corner."""
        name = data.get("flight_name", "")
        if name:
            ax.text(0.98, 0.02, name,
                    transform=ax.transAxes,
                    ha="right", va="bottom", fontsize=8, alpha=0.7, zorder=10,
                    bbox=dict(facecolor="white", alpha=0.8, edgecolor="#EAEAEA",
                              boxstyle="round,pad=0.2"))

    # ── Speed: Fixed Cage (top-left) ──
    ax = axes[0, 0]
    ax.plot(ekf_fix["t"], fix_data["df_mocap"]["speed"].values,
            "b-", lw=0.8, alpha=0.7, label="MoCap SG")
    ax.plot(ekf_fix["t"], ekf_fix["speed"],
            "green", lw=1.3, alpha=0.9, label="EKF (PX4)")
    ax.set_ylim(-0.05, 0.85)
    ax.set_yticks(np.arange(0, 0.9, 0.2))
    ax.set_title(f"Fixed Cage — Representative (MoCap ≈ {ekf_fix['rate']} Hz)",
                 fontsize=12, color="#cc3300")
    ax.set_ylabel("Speed [m/s]")
    ax.set_xlim(t_min_fix, t_max_fix)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    _flight_label(fix_data, ax)

    # ── Speed: Rotating Cage (top-right) ──
    ax = axes[0, 1]
    ax.plot(ekf_rot["t"], rot_data["df_mocap"]["speed"].values,
            "b-", lw=0.8, alpha=0.7, label="MoCap SG")
    ax.plot(ekf_rot["t"], ekf_rot["speed"],
            "green", lw=1.3, alpha=0.9, label="EKF (PX4)")
    ax.set_ylim(-0.05, 0.85)
    ax.set_yticks(np.arange(0, 0.9, 0.2))
    ax.set_title(f"Rotating Cage — Representative (MoCap ≈ {ekf_rot['rate']} Hz)",
                 fontsize=12, color="#006600")
    ax.set_ylabel("Speed [m/s]")
    ax.set_xlim(t_min_rot, t_max_rot)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    _flight_label(rot_data, ax)

    # ── Per-axis: Fixed Cage (bottom-left, fixed Y: -0.4 to 0.6) ──
    ax = axes[1, 0]
    ax.plot(ekf_fix["t"], fix_data["df_mocap"]["vx"].values, "b-", lw=0.7,
            alpha=0.6, label="MoCap VX")
    ax.plot(ekf_fix["t"], fix_data["df_mocap"]["vy"].values, "r-", lw=0.7,
            alpha=0.6, label="MoCap VY")
    ax.plot(ekf_fix["t"], fix_data["df_mocap"]["vz"].values, "orange", lw=0.7,
            alpha=0.6, label="MoCap VZ")
    ax.plot(ekf_fix["t"], ekf_fix["vx"], "green", lw=1.2, alpha=0.85,
            label="EKF VX")
    ax.plot(ekf_fix["t"], ekf_fix["vy"], "magenta", lw=1.2, alpha=0.85,
            ls="--", label="EKF VY")
    ax.plot(ekf_fix["t"], ekf_fix["vz"], "brown", lw=1.2, alpha=0.85,
            ls=":", label="EKF VZ")
    ax.set_ylim(-0.4, 0.6)
    ax.set_xlim(t_min_fix, t_max_fix)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper right", fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    _flight_label(fix_data, ax)

    # ── Per-axis: Rotating Cage (bottom-right, fixed Y: -0.4 to 0.6) ──
    ax = axes[1, 1]
    ax.plot(ekf_rot["t"], rot_data["df_mocap"]["vx"].values, "b-", lw=0.7,
            alpha=0.6, label="MoCap VX")
    ax.plot(ekf_rot["t"], rot_data["df_mocap"]["vy"].values, "r-", lw=0.7,
            alpha=0.6, label="MoCap VY")
    ax.plot(ekf_rot["t"], rot_data["df_mocap"]["vz"].values, "orange", lw=0.7,
            alpha=0.6, label="MoCap VZ")
    ax.plot(ekf_rot["t"], ekf_rot["vx"], "green", lw=1.2, alpha=0.85,
            label="EKF VX")
    ax.plot(ekf_rot["t"], ekf_rot["vy"], "magenta", lw=1.2, alpha=0.85,
            ls="--", label="EKF VY")
    ax.plot(ekf_rot["t"], ekf_rot["vz"], "brown", lw=1.2, alpha=0.85,
            ls=":", label="EKF VZ")
    ax.set_ylim(-0.4, 0.6)
    ax.set_xlim(t_min_rot, t_max_rot)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Velocity [m/s]")
    ax.legend(loc="upper right", fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    _flight_label(rot_data, ax)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"[INFO]  Dual EKF comparison saved to:  {output_path}")
    plt.show()
    print(f"✅ Dual EKF comparison: "
          f"Fixed ({ekf_fix['rate']} Hz) | Rotating ({ekf_rot['rate']} Hz)")


def plot_full_loop_geometry_from(angle_deg=45, output_path=None, show_plot=True):
    """Theoretical full-loop geometry — wrapper for notebook.

    Calls ``plot_full_loop_geometry()`` from ``kin_plot_trajectory`` with
    the selected angle (45 or 75).  No flight data needed.
    """
    plot_full_loop_geometry(angle_deg=angle_deg, output_path=output_path, show_plot=show_plot)
