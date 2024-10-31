# F1TENTH Gym Environment

To install the F1TENTH Gym environment, please visit the official repository:

**[F1TENTH Gym Environment Installation](https://github.com/f1tenth/f1tenth_gym_ros.git)**

Follow the instructions provided in that repository to set up the environment and install all required dependencies.

# ROS 2 F1Tenth Simulation Launch Instructions (Concise)

This guide provides instructions on how to navigate to your ROS 2 simulation workspace, source it, launch the simulation, and control it using the keyboard.

## Steps to Launch the Simulation

1. **Navigate to the simulation workspace**:
   
   ```bash
   cd sim_ws/

2. **Source the workspace:**:
   
   ```bash
   . install/setup.bash

3. **Launch the simulation:**:
   
   ```bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py

4. **Enable Keyboard Teleoperation (optional):**:
   To control the simulation with the keyboard, first make sure kb_teleop is set to True in the sim.yaml configuration file. Then, open a new terminal, navigate to your workspace, source it, and run:
   
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   
## Control Instructions:
- Press `i` to move `forward`.
- Press `u` or `o` to turn `left` or `right` while moving forward.
- Press `,` to move backward.
- Press `m` or `.` to turn `left` or `right` while moving backward.
- Press `k` to stop.
   
