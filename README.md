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


## Dependencies
- **ROS 2 (Humble or newer)**
- **Python 3.8+**
- **sensor_msgs** and **geometry_msgs**

Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

---
