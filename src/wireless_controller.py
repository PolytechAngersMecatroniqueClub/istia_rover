#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from evdev import InputDevice, categorize, ecodes, ff, list_devices
import sys as sys
import os as os
import time as time

DEFAULT_DEVADD = "b8:27:eb:7a:9c:dc"
DEFAULT_DEVNAME = "Wireless Controller"

rospy.init_node('wireless_controller')

devname = DEFAULT_DEVNAME
devadd = DEFAULT_DEVADD

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
        print("<"+str(devname)+"> is connected to <"+str(device.path)+">")
        gamepad = device

if gamepad == None: # this means the controller was not found, the programm has to stop
    print("<"+str(devname)+"> not found")
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
#evdev takes care of polling the controller in a loop

print(gamepad.capabilities(verbose=True))

# to make the pad rumbled:
rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
effect_type = ff.EffectType(ff_rumble_effect=rumble)
duration_ms=1000

effect = ff.Effect(
    ecodes.FF_RUMBLE, -1, 0,
    ff.Trigger(0,0),
    ff.Replay(duration_ms, 0),
    ff.EffectType(ff_rumble_effect=rumble)
)

repeat_count = 1
effect_id = gamepad.upload_effect(effect)
gamepad.write(ecodes.EV_FF, effect_id, repeat_count)

flat_rosbag_started = False

for event in gamepad.read_loop():
    #filters by event type
    if event.type == ecodes.EV_KEY:
        if (event.code == 305) and (event.val == 1): # The round key has been pressed
            gamepad.write(ecodes.EV_FF, effect_id, repeat_count)
            current_time = time.strftime("%y-%m-%d_%H-%M-%S")
            os.system('tmux send-keys -t \'rosbag\' \'' + current_time + '.bag\' ')
            print("ROSBAG started")

        rospy.loginfo(event)
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
