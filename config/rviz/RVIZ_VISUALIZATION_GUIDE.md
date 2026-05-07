# Real-Time Drone Odometry Visualization & Flight Recording

## Quick Start

### On Your Laptop (Real-time monitoring)
```bash
cd ~/pi_drone_sshfs
./launch_rviz_laptop.sh
```

RViz will display:
- **Fixed Frame Axes** (white): origin reference
- **Grid**: XY ground plane  
- **Green Axes**: drone orientation (FRD frame)
- **Purple Trail**: flight path history (up to 1000 points)
- **Updates at 30 Hz** from the mocap → PX4 chain

### On Pi (During flight - in separate terminal)
```bash
cd /home/ws
./record_flight_bag.sh ~/flight_bags/my_test_flight
```

This records:
- `/fmu/in/vehicle_visual_odometry` — mocap pose sent to PX4
- `/fmu/out/vehicle_odometry` — PX4 fused odometry
- `/fmu/out/vehicle_local_position` — PX4 position estimate
- `/fmu/out/vehicle_attitude` — PX4 attitude
- `/poses` — raw mocap data (for debugging frame transforms)

## Post-Flight Analysis

### Replay & Visualize
```bash
# Terminal 1: Replay the bagfile
ros2 bag play ~/flight_bags/my_test_flight

# Terminal 2: Start RViz with same config
cd ~/pi_drone_sshfs
./launch_rviz_laptop.sh
```

The trajectory will replay at real-time speed. You can:
- Pause/play with spacebar
- Step through frames
- Inspect individual topics with `ros2 topic echo`

### Extract Trajectory Data
```bash
# Convert bag to CSV for analysis in Python/MATLAB
cd ~/flight_bags/my_test_flight
python3 << 'EOF'
import sqlite3
import numpy as np

# Extract position data from bag
db = sqlite3.connect(my_test_flight/metadata.db)
cursor = db.cursor()

# Query vehicle_visual_odometry messages
cursor.execute("""
    SELECT * FROM messages 
    WHERE topic_id IN (SELECT id FROM topics WHERE name = '/fmu/in/vehicle_visual_odometry')
    ORDER BY timestamp
""")

positions = []
for row in cursor.fetchall():
    # Parse the serialized message...
    # Extract x, y, z, timestamp
    pass

np.savetxt('trajectory.csv', positions, delimiter=',', 
            header='timestamp,x,y,z,qx,qy,qz,qw')
EOF
```

## Troubleshooting

### RViz shows "No transform" error
- Ensure `/fmu/in/vehicle_visual_odometry` is being published
- Check: `ros2 topic echo /fmu/in/vehicle_visual_odometry --once`
- Verify ROS_DOMAIN_ID matches (should be 0 on both Pi and laptop)

### Recording stops early
- Check disk space on Pi: `df -h /home`
- Monitor network: `ros2 doctor`

### Bag replay is slow
- Reduce playback speed: `ros2 bag play -r 0.5 <bagfile>` (half speed)
- Or step manually with `--clock` for frame-by-frame inspection

## Files Created

- `config/rviz/drone_odometry.rviz` — RViz config (FRD odometry + trajectory)
- `launch_rviz_laptop.sh` — Laptop launch script (sets env, starts RViz)
- `record_flight_bag.sh` — Pi recording script (captures all relevant topics)
