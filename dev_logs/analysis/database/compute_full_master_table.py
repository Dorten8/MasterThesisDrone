import sqlite3
import numpy as np

def compute_master_metrics():
    conn = sqlite3.connect('dev_logs/analysis/experiments_summary.db')
    cursor = conn.cursor()
    
    # We will compute stats for: Overall, 45°, 75°
    categories = ['Overall', '45°', '75°']
    
    results = {
        'Rotating Cage': {'Overall': {}, '45°': {}, '75°': {}},
        'Fixed Cage': {'Overall': {}, '45°': {}, '75°': {}}
    }
    
    query = """
    SELECT 
        condition,
        nom_sp_x,
        AVG(imu_peak_accel_z),
        AVG(max_dev_after),
        AVG(imu_gyro_energy),
        AVG(max_actuator_output),
        AVG(motor_max_after),
        AVG(voltage_drop_rate_v_per_min),
        AVG(capacity_drain_rate_pct_per_min),
        AVG(active_flight_time_sec),
        AVG(recovery_area),
        AVG(path_spread_sdld),
        AVG(roll_rate_error_rms),
        AVG(pitch_rate_error_rms),
        AVG(yaw_rate_error_rms),
        AVG(allocator_saturation_duration_sec),
        AVG(max_unallocated_torque),
        AVG(thrust_setpoint_achieved_pct),
        AVG(imu_ax_spread_impact),
        AVG(imu_ay_spread_impact),
        AVG(imu_az_spread_impact),
        AVG(imu_ax_spread_regular),
        AVG(imu_ay_spread_regular),
        AVG(imu_az_spread_regular),
        
        -- Also getting stdevs where possible, using variance or approx via python. 
        -- To keep it simple, we'll fetch all rows and compute mean/std in Python.
        flight_name
    FROM flights_summary
    WHERE impact_detected = 1
    GROUP BY condition, nom_sp_x
    """
    
    # Wait, it's better to fetch ALL raw data and compute mean ± SD in python!
    cursor.execute("""
    SELECT 
        condition, nom_sp_x, imu_peak_accel_z, max_dev_after, imu_gyro_energy,
        max_actuator_output, motor_max_after, voltage_drop_rate_v_per_min,
        capacity_drain_rate_pct_per_min, active_flight_time_sec, recovery_area,
        path_spread_sdld, roll_rate_error_rms, pitch_rate_error_rms, yaw_rate_error_rms,
        allocator_saturation_duration_sec, max_unallocated_torque, thrust_setpoint_achieved_pct,
        imu_ax_spread_impact, imu_ay_spread_impact, imu_az_spread_impact,
        imu_ax_spread_regular, imu_ay_spread_regular, imu_az_spread_regular
    FROM flights_summary
    WHERE impact_detected = 1
    """)
    rows = cursor.fetchall()
    conn.close()

    # Organize raw data
    raw_data = {
        'Rotating Cage': {'Overall': [], '45°': [], '75°': []},
        'Fixed Cage': {'Overall': [], '45°': [], '75°': []}
    }
    
    for r in rows:
        cond = r[0]
        angle_approx = abs(r[1])
        angle_cat = '45°' if angle_approx < 0.6 else '75°'
        
        row_dict = {
            'peak_accel_z': r[2], 'max_dev': r[3], 'gyro_energy': r[4],
            'actuator': r[5], 'rpm': 2000 + 10000 * r[6] if r[6] is not None else None,
            'voltage': r[7], 'capacity': r[8], 'active_time': r[9],
            'recovery': r[10], 'sdld': r[11],
            'roll_rms': r[12], 'pitch_rms': r[13], 'yaw_rms': r[14],
            'alloc_sat': r[15], 'max_unalloc': r[16], 'thrust_pct': r[17],
            'ax_impact': r[18], 'ay_impact': r[19], 'az_impact': r[20],
            'ax_reg': r[21], 'ay_reg': r[22], 'az_reg': r[23]
        }
        
        raw_data[cond]['Overall'].append(row_dict)
        raw_data[cond][angle_cat].append(row_dict)

    def calc_stats(lst, key):
        vals = [item[key] for item in lst if item[key] is not None]
        if not vals:
            return None, None
        return np.mean(vals), np.std(vals)

    def format_val(mean, std, decimals=3):
        if mean is None: return "N/A"
        format_str = f"{{:.{decimals}f}} ± {{:.{decimals}f}}"
        return format_str.format(mean, std)

    def format_pct(mean_rot, mean_fix):
        if mean_rot is None or mean_fix is None or mean_fix == 0:
            return "N/A"
        pct = ((mean_rot - mean_fix) / abs(mean_fix)) * 100.0
        sign = "+" if pct >= 0 else ""
        return f"{sign}{pct:.1f}%"

    print("### 🏆 High-Fidelity Expanded Master Comparison Metrics Table\n")
    
    # Print the table header
    print("| Metric | Rotating Cage (Overall) | Fixed Cage (Overall) | Δ | Rotating (45°) | Fixed (45°) | Δ | Rotating (75°) | Fixed (75°) | Δ |")
    print("|--------|-------------------------|----------------------|---|----------------|-------------|---|----------------|-------------|---|")
    
    metrics_list = [
        ("Peak Deceleration Z (g)", "peak_accel_z", 3),
        ("Maximum Attitude Deviation (m)", "max_dev", 3),
        ("Integrated Rotational Energy (rad)", "gyro_energy", 3),
        ("Max Actuator Output (%)", "actuator", 2),
        ("Average Commanded Motor Speed (RPM)", "rpm", 1),
        ("Voltage Drop Rate (V/min)", "voltage", 3),
        ("Capacity Drain Rate (%/min)", "capacity", 3),
        ("Active Flight Time (s)", "active_time", 1),
        ("Recovery Area (m²)", "recovery", 3),
        ("Path Spread SDLD (m)", "sdld", 3),
        ("Attitude Rate Roll RMS (rad/s)", "roll_rms", 3),
        ("Attitude Rate Pitch RMS (rad/s)", "pitch_rms", 3),
        ("Attitude Rate Yaw RMS (rad/s)", "yaw_rms", 3),
        ("Allocator Saturation Duration (s)", "alloc_sat", 3),
        ("Max Unallocated Torque (N·m)", "max_unalloc", 3),
        ("Thrust Setpoint Achieved (%)", "thrust_pct", 2),
        ("IMU Accel Spread X (Impact) (g)", "ax_impact", 3),
        ("IMU Accel Spread Y (Impact) (g)", "ay_impact", 3),
        ("IMU Accel Spread Z (Impact) (g)", "az_impact", 3),
        ("IMU Accel Spread X (Regular) (g)", "ax_reg", 3),
        ("IMU Accel Spread Y (Regular) (g)", "ay_reg", 3),
        ("IMU Accel Spread Z (Regular) (g)", "az_reg", 3),
    ]
    
    for label, key, dec in metrics_list:
        row_str = f"| {label} |"
        for cat in categories:
            rot_mean, rot_std = calc_stats(raw_data['Rotating Cage'][cat], key)
            fix_mean, fix_std = calc_stats(raw_data['Fixed Cage'][cat], key)
            
            rot_str = format_val(rot_mean, rot_std, dec)
            fix_str = format_val(fix_mean, fix_std, dec)
            pct_str = format_pct(rot_mean, fix_mean)
            
            row_str += f" {rot_str} | {fix_str} | {pct_str} |"
        print(row_str)

if __name__ == '__main__':
    compute_master_metrics()
