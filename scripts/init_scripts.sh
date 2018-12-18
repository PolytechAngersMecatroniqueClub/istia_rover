#!/bin/bash
# this script is called "init_scripts.sh"
# the idea is to put the scripts in the folder /home/pi/bin




read -p "This will copy the scripts from the git repository to the bin directory, do you want to continue ? (y/n)" input

if [ $input = "y" ] 
then 
    if [ ! -d /home/pi/bin ];
    then
        mkdir /home/pi/bin
        echo "The bin directory does not exists, it has been created (/home/pi/bin)"
    fi

    if [ ! -d /home/pi/logs ];
    then
        mkdir /home/pi/logs
        echo "The logs directory does not exists, it has been created (/home/pi/logs)"
    fi

    yes | cp -f /ros_catkin_ws/scripts/* /home/pi/bin/
    echo "Scripts copied from git to bin"
    # the init_scripts should not be done from the /home/pi/bin directory
    # this is done to avoid unwanted overwritting
    rm /home/pi/bin/init_scripts.sh
    echo "the file init_scripts has been removed from the bin directory because it should only be execute in the git directory"
elif [ $input = "n" ]
then
    echo "Abord!"
else
    echo "the choice \"$input\" is not valid"
fi




