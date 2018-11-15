#!/bin/bash
# this script is called "update_scripts.sh"
# The idea is to copy the scripts into the github repository in oder to update them if needed
# note that it is assumed that the istia_rover github repositirory is at /home/pi/ros_catkin_ws/src/istia_rover !

yes | cp -f /home/pi/bin/* /home/pi/ros_catkin_ws/src/istia_rover/scritps/*


