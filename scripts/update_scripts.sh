#!/bin/bash
# this script is called "update_scripts.sh"
# The idea is to copy the scripts into the github repository in oder to update them if needed
# note that it is assumed that the istia_rover github repositirory is at /home/pi/ros_catkin_ws/src/istia_rover !


read -p "This will copy the scripts from the bin directory to git repository, do you want to continue ? (y/n)" input

if [ $input = "y" ] 
then 
    yes | cp -f /home/pi/bin/* /home/pi/ros_catkin_ws/src/istia_rover/scripts/
    echo "Scripts copied from bin to git"
elif [ $input = "n" ]
then
    echo "Abord!"
else
    echo "the choice \"$input\" is not valid"
fi


