#!/bin/bash
# this script is called "init_scripts.sh"
# the idea is to put the scripts in the folder /home/pi/bin

if [ ! -d /home/pi/bin ];
then
    mkdir /home/pi/bin
fi

if [ ! -d /home/pi/logs ];
then
    mkdir /home/pi/logs
fi

mv * /home/pi/bin/

# the init_scripts should not be done from the /home/pi/bin directory
# this is done to avoid unwanted overwritting
rm /home/pi/bin/init_scripts
