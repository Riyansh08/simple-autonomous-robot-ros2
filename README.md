# Simple Autonomous Robot (ROS 2)

This project simulates a simple autonomous robot equipped with LIDAR-based obstacle avoidance, developed using ROS 2. The system is designed to navigate autonomously by detecting obstacles, performing proportional control, and avoiding collisions.

---

## Features
- **LIDAR Sensor Simulation** – Simulates real-world LIDAR behavior with noise and adjustable parameters.
- **Obstacle Avoidance** – Dynamic adjustment of linear and angular speeds based on obstacle distance.
- **Recovery Behavior** – Reverses or re-orients the robot upon encountering close-range obstacles.
- **Dynamic Configuration** – YAML-based parameter tuning for sensor range, speed, and avoidance thresholds.
- **Fault Tolerance** – Automatically restarts nodes if they crash during runtime.
- **Flexible Launch System** – Parameterized launch files for easy customization and real-time adjustments.

---

## Project Structure
```text
ros2_ws/
└── src/
    └── simple_autonomous_robot/
        ├── config/
        │   ├── lidar_params.yaml
        │   └── controller_params.yaml
        ├── simple_autonomous_robot/
        │   ├── __init__.py
        │   ├── lidar_sensor.py
        │   ├── robot_controller.py
        │   └── movement.py
        ├── launch/
        │   └── robot_launch.py
        ├── resource/
        │   └── simple_autonomous_robot
        ├── setup.py
        ├── package.xml
        └── setup.cfg
```

---

## How to Build and Run

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

### 2. Launch the Robot
```bash
ros2 launch simple_autonomous_robot robot_launch.py
```

---

### 3. Customize Parameters at Launch
```bash
ros2 launch simple_autonomous_robot robot_launch.py linear_speed:=0.5 angular_speed:=1.0
```

---

## YAML Configuration Files

### LIDAR Configuration (`lidar_params.yaml`)
```yaml
/**:
  lidar_sensor:
    ros__parameters:
      range_min: 0.1
      range_max: 10.0
      angle_min: -1.57
      angle_max: 1.57
      angle_increment: 0.01
      noise_level: 0.05
```

### Controller Configuration (`controller_params.yaml`)
```yaml
/**:
  robot_controller:
    ros__parameters:
      linear_speed: 0.3
      angular_speed: 0.7
      obstacle_distance_threshold: 1.0
      recovery_behavior: true
```

---

## Dependencies
- **ROS 2 (Humble or newer)**
- **Python 3.8+**
- **sensor_msgs** and **geometry_msgs**

Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

---
