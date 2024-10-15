# ME-35 Intro to ROS Lesson

This repository contains code for the **ME-35** intro lesson on **ROS2**. The goal of this lesson is to provide an introduction to ROS2 programming through a digital visualization of a robot without having to install ROS locally to use existing packages like RViz or Gazebo. This means a virtual ROS environment can be hosted through a Jupyter Hub so students can still learn ROS without a physical robot present and without having to download any software. Then, they can take turns running their programs on the real arm and see their simulation come to life!

## Files

### `MQTT_UR_Sim.m`
- This MATLAB function is the core program for launching and controlling the simulated arm using **MQTT**.
- **Important**: Be sure to edit the MQTT client and topic names within the script to avoid conflicts with other devices or instances.

### `MainActivities.ipynb`
- This jupyter notebook introduces the main ROS2 concepts like topics, nodes, publish/subscribing, services, actions and parameters.

### `ArmSimDemo.ipynb`
- This jupyter notebook introduces simple joint angle control of the simulated arm launched by MQTT_UR_Sim.m
- In the future this same notebook can be used for real robot control as well to see how the two are similar

### `my_sim_ur_pkg`
- This directory is the ROS2 package that launches arm simulator action server nodes for N groups using a launch file where the number of groups can be set
- To run this package you can build it in your workspace using:
```bash
# build package (could use --symlink-install to auto update changes but skipped for this)
sudo colcon build --packages-select my_sim_ur_pkg

# source the install from ws 
source install/setup.bash

# spins multiple nodes based on the input in the launch python file
ros2 launch my_sim_ur_pkg launch_sim_nodes.py
```
### `example_sim_control.py`
- A simple Python script designed to test the MQTT-based configuration of the simulated arm.
- **Note**: Make sure to update the MQTT client and topic names in this script to match the names used in the MATLAB program.
