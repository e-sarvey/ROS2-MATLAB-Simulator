# ME-35 Intro to ROS Lesson

This repository contains code for the **ME-35** intro lesson on **ROS2**. The goal of this lesson is to provide an introduction to ROS2 programming through a digital visualization of a robot without having to install ROS locally to use existing packages like RViz or Gazebo. This means a virtual ROS environment can be hosted through a Jupyter Hub so students can still learn ROS without a physical robot present and without having to download any software. Then, they can take turns running their programs on the real arm and see their simulation come to life!

## Files

### `MQTT_UR_Sim.m`
- This MATLAB script is the core program for launching and controlling the simulated arm using **MQTT**.
- **Important**: Be sure to edit the MQTT client and topic names within the script to avoid conflicts with other devices or instances.

### `example_sim_control.py`
- A simple Python script designed to test the MQTT-based configuration of the simulated arm.
- **Note**: Make sure to update the MQTT client and topic names in this script to match the names used in the MATLAB program.

## Future Updates
- The **ROS2 Simulation Action Server** will be added in future updates. This package creates a ROS2 node for controlling the simulated arm as an action server. It will be integrated into class exercises for hands-on learning.
