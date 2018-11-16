#!/usr/bin/env python

from evdev import InputDevice, categorize, ecodes, list_devices
import os as os
import sys as sys
from time import sleep

DEFAULT_DEVADD = "b8:27:eb:7a:9c:dc"
DEFAULT_DEVNAME = "Wireless Controller"
DEFAULT_TMUX = ""

devname = DEFAULT_DEVNAME
devadd = DEFAULT_DEVADD

os.system('echo "looking for the wireless controller"')
os.system('/home/pi/bin/ihm.py -c blue -t "waiting ctrl"')

# we deal with the command line arguments
if len(sys.argv) > 1: # if we have at least one argument in the commande line
    if (sys.argv[1] == "-h") or (sys.argv[1] == "help"): # it can be the ask for help
        print("USAGE: "+str(sys.argv[0])+" [device_name] [device_add]")
        print(" - default device_name : "+str(DEFAULT_DEVNAME))
        print(" - default device_add : "+str(DEFAULT_DEVADD))
        exit()
    else: # or ovewriting the default device name
        devname = sys.argv[1]
if len(sys.argv) > 2: # if there is a second argument, it is the device address
    devadd = sys.argv[2]

gamepad = None

while gamepad == None:

    # we get all the connected devices
    devices = [InputDevice(path) for path in list_devices()]

    # we loop over those devices looking for the wireless controller
    for device in devices:
        print(device)
        if (device.phys == devadd) and (device.name == devname) :
            print("<"+str(devname)+"> is connected to <"+str(device.path)+">")
            gamepad = device

    if gamepad == None: # this means the controller was not found, the programm has to stop
        os.system('echo "controller not found - sleeping 5s"')
        sleep(5)

os.system('/home/pi/bin/ihm.py -c green -t "ctrl connected"')

os.system('echo "wireless controller found - starting the wireless_controller ros node"')
os.system('echo "sending the source command to the tmux window"')
os.system("tmux send-keys -t 'ctrlnode' 'rosrun istia_rover wireless_controller.py' C-m")
os.system('echo "command sent, nothing left to do"')
os.system('echo starting the motorhat ros node')
os.system("tmux send-keys -t 'motorhat' 'roslaunch istia_rover wireless_controller.launch' C-m")
# I do not know why it does not work the first time... so try it a second time...
os.system("tmux send-keys -t 'motorhat' 'roslaunch istia_rover wireless_controller.launch' C-m")

