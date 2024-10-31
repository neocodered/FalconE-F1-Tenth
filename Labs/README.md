# F1Tenth Course Workspace

This directory contains all the workspace files created and used during the course labs for the **F1Tenth Course**. It includes the necessary code, configurations, and resources developed throughout the course, enabling you to build and enhance autonomous systems using the F1Tenth platform.

## Contents

The repository includes:

1. **Lab Codes**: All the code files created during various labs in the course. This covers implementations, algorithms, and simulation models designed for autonomous racing systems.
2. **Configurations**: Relevant configuration files needed to set up and run simulations or experiments with F1Tenth. These include sensor setups, vehicle parameters, and system configurations.
3. **Resources**: Additional resources such as documentation, helper scripts, and assets required for specific lab tasks or projects.
4. **Workspace Files**: Complete ROS workspace structure used in the F1Tenth environment, containing all the necessary files to execute the labs.

## Running the ROS 2 Launch File

Follow these steps to navigate to the workspace, source it, and launch the nodes:

1. **Navigate to the workspace**:
   ```bash
   cd lab1_ws/

2. **Source the workspace:**:
   ```bash
   . install/setup.bash

3. **Run the launch file:**:
   ```bash
   ros2 launch lab1_pkg lab1_launch.py
