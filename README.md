This repo contains code for an ME-35 intro to ROS lesson.

MQTT_UR_Sim.m is the main MATLAB script that launches the simulated arm and controls it over MQTT. Edit the MQTT client and topic names to avoid conflicts!

example_sim_control.py is a simple python program that can be editted to test the configuration of the simulated arm over MQTT. Edit the MQTT client and topic names to match the names in the MATLAB program.

Future updates will include the ROS2 package that creates a ROS2 node for controlling the simulation that will be used in class.
