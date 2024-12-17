# Safety Node for Emergency Braking in ROS 2

This is a ROS 2 package that implements an **emergency braking system** using LIDAR data and vehicle odometry. The node calculates the **Time-To-Collision (TTC)** and stops the vehicle when a collision is imminent.

---

## Overview
The `safety_node` subscribes to:
- `/scan` (LIDAR data)
- `/ego_racecar/odom` (vehicle odometry)

The node publishes to:
- `/drive` with an **AckermannDriveStamped** message to command the vehicle to stop if the TTC falls below a threshold.

---

## Dependencies
This project depends on the following ROS 2 message packages:
- `rclpy`
- `ackermann_msgs`
- `sensor_msgs`
- `nav_msgs`
  
Install dependencies using:
```bash
sudo apt install ros-humble-ackermann-msgs ros-humble-sensor-msgs ros-humble-nav-msgs
