#!/bin/bash
# this script is called "init_scripts.sh"
# the idea is to put the scripts in the folder /home/pi/bin

if [ ! -d /home/pi/bin]
    mkdir /home/pi/bin
fi

if [ ! -d /home/pi/logs]
    mkdir /home/pi/logs
fi

mv * /home/pi/bin/
