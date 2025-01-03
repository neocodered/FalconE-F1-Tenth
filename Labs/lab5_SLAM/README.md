# TurtleBot3 Mapping and Navigation Setup

This guide provides the steps to set up and run a TurtleBot3 in a simulated environment for mapping and navigation using ROS2.

## Prerequisites

- ROS2 Humble installed
- TurtleBot3 simulation packages installed
- Proper environment setup for TurtleBot3

## Steps

### 1. Launch the TurtleBot3 World
Open a terminal and run the following command to start the simulation in Gazebo:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. Launch the Navigation2 Stack
Open a second terminal and run the Navigation2 stack with simulation time enabled:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

3. Launch the SLAM Toolbox
In a third terminal, run the SLAM toolbox with simulation time enabled:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

4. Visualize the Mapping Process
Open a fourth terminal and launch RViz2 to visualize the mapping process:

```bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

5. Control the Robot
In a fifth terminal, run the teleoperation node to control the TurtleBot3 and explore the environment:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

6. Save the Map
After mapping the environment, stop the teleoperation node and save the map using the following command:
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

The map will be saved as `my_map.yaml` and `my_map.pgm` in the current directory.
