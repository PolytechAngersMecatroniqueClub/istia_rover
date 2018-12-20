#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from evdev import InputDevice, categorize, ecodes, ff, list_devices
import sys as sys
import os as os
import time as time
import subprocess as sp


DEFAULT_DEVADD = "b8:27:eb:7a:9c:dc"
DEFAULT_DEVNAME = "Wireless Controller"
DEFAULT_USB = "/media/pi/ROSBAG_KEY/"

rospy.init_node('wireless_controller')

devname = DEFAULT_DEVNAME
devadd = DEFAULT_DEVADD
usblink = DEFAULT_USB

# we deal with the command line arguments
if len(sys.argv) > 1: # if we have at least one argument in the commande line
    if (sys.argv[1] == "-h") or (sys.argv[1] == "help"): # it can be the ask for help
        print("USAGE: <"+str(sys.argv[0])+" device_name device_add>")
        print(" - default device_name : "+str(DEFAULT_DEVNAME))
        print(" - default device_add : "+str(DEFAULT_DEVADD))
        exit()
    else: # or ovewriting the default device name
        devname = sys.argv[1]
if len(sys.argv) > 2: # if there is a second argument, it is the device address
    devadd = sys.argv[2]


# we get all the connected devices
devices = [InputDevice(path) for path in list_devices()]
gamepad = None

# we loop over those devices looking for the wireless controller
for device in devices:
    if (device.phys == devadd) and (device.name == devname) :
        rospy.loginfo("<"+str(devname)+"> is connected to <"+str(device.path)+">")
        gamepad = device

if gamepad == None: # this means the controller was not found, the programm has to stop
    rospy.logerror("<"+str(devname)+"> not found")
    exit()

cmd_pub = rospy.Publisher('/wireless_controller/cmd_vel',
                          Twist, queue_size=1,
                          latch=True)

def send(values):
    msg = Twist()
    msg.linear.x = values[0]
    msg.angular.z = values[1]
    cmd_pub.publish(msg)

r = rospy.Rate(50)

x = 0
z = 0
x_old = 0
z_old = 0

# evdev takes care of polling the controller in a loop
# print(gamepad.capabilities(verbose=True)
# to make the pad rumbled:

flag_rosbag_started = False

os.system('/home/pi/bin/ihm.py -c green -t "ctrl node started"')

for event in gamepad.read_loop():
    #filters by event type
    if event.type == ecodes.EV_KEY:
        print(event)
        if (event.code == 305) and (event.value == 1): # The round key has been pressed
            if flag_rosbag_started:
                rospy.logwarn("Rosbag already running, you need to stop it before starting a new one")
            else:
                # repeat_count = 1
                # effect_id = gamepad.upload_effect(effect)
                # gamepad.write(ecodes.EV_FF, effect_id, repeat_count)
                if os.path.isdir(usblink):
                    os.system('sudo chown pi '+usblink)
                    proc = sp.Popen('ls ' + usblink + ' | wc -l', shell=True, stdout=sp.PIPE)
                    answer = int(proc.stdout.read())
                    num_file = answer + 1
                    print(answer, num_file)
                    os.system('tmux send-keys -t \'rosbag\' \'rosbag record -O ' + usblink + str(num_file) + '.bag -a\' C-m')
                    rospy.loginfo("ROSBAG started")
                    os.system('/home/pi/bin/ihm.py -c white -t "recording"')
                    flag_rosbag_started = True
                else:
                    os.system('/home/pi/bin/ihm.py -c red -t "usb not detected"')

        elif (event.code == 304) and (event.value == 1): # The cross key has been pressed
                os.system('tmux send-keys -t \'rosbag\'  C-c')
                os.system('/home/pi/bin/ihm.py -c red -t "not recording"')
                rospy.loginfo('ROSBAG stopped')
                flag_rosbag_started = False
                time.sleep(2)

        elif (event.code == 307) and (event.value == 1): # The triangle key has been pressed
                os.system('mkdir ' + usblink + 'logs')
                os.system('yes | cp -f ~/logs/* ' + usblink + 'logs/')
                rospy.loginfo('Scripts copied')

    elif  event.type == ecodes.EV_ABS:
        flagupdate = False
        if(event.code == 0):
            z = -int(((event.value - 125)*3/130.0)*10)/10.0
            flagupdate = True
        elif(event.code == 1):
            x = -int(((event.value - 125)*3/130.0)*10)/10.0
            flagupdate = True
    if (x_old != x) or (z_old != z):
        send([x,z])
        x_old = x
        z_old = z
    if rospy.is_shutdown():
        break;
