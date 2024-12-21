# Wall Follow Package

## Overview
The "wall_follow" package is designed for ROS2-based autonomous car navigation. This package implements essential nodes for:
- Driving the car manually for performance testing and debugging - "car_drive_node"
- Following the left-side wall - "reactive_node"
- Implementing the "Follow the Gap" algorithm for reactive navigation - "wall_follow_node"

## Directory Setup
Place the "wall_follow" package inside your ROS2 workspace:
```bash
  cd ~/sim_ws/src
```

## Building the Package
After placing the package inside the workspace, build it using colcon:
```bash
cd ~/sim_ws
colcon build
source install/setup.bash
```


