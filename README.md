# Istia Rover
This repository contains the ROS package used on the rover and some usefull scripts an explanation

# The physical parts of the rover
The istia rover is based on:
 - a 4 Wheels Scout Platform Robot Kit from ServoCity (https://www.servocity.com/scout)
 - a raspberry pi 3B+
 - a motor hat stepper motor shield from adafruit (https://www.adafruit.com/product/2348)
 - two alimentations boards UNI-REG (https://www.lextronic.fr/conversion-tension/29859-module-alimentation-1-8-2-7-3-3-4-5-12vcc.html) - One for the motors hat shield (12v) and one for the raspberry (5v)
 - a LiDAR hokuyo utm-30lx (with an home made power board for power supply the sensor)
 - a TSL2561 lux sensor from adafruit (https://www.adafruit.com/product/439)
 - an Adafruit Pi Plate (16x2 Character LCD + Keypad) (https://learn.adafruit.com/adafruit-16x2-character-lcd-plus-keypad-for-raspberry-pi/overview)

# The ROS istia_rover package
This package contains specifics nodes to run the rover and the sensors. Note that the source code of the nodes are in the istia_rover/src folder. Note that it has been tested for ros-kinetic

## motor_hat_twist_node.cpp
This node allows to convert a twist message to 4 PWM commands for the motors. It is based on the motor_hat_node from matpalm (https://github.com/matpalm/ros-motorhat-node).

The default listened topic is /default/cmd_vel
Using a launch file it is possible to define:
 - the topic name for the twist command
 - the addresses of the motors (the wirering motor/board)
 - the considered max twist value and the considered max speed

Examples of launch file are presented later. It is assumed that only one motor shield is used and that the I2C address is 0x60.

## tsl2561_node.cpp
This node allows to handle the TSL2561 lux sensor. This sensor is directly connected to the I2C of the rapsberry pi with 0x39 as its I2C address.

The node publishes three sensor_msgs::Illuminance messages (tsl2561/full_spectrum, tsl2561/infrared, tsl2561/visible) at a 5Hz frequency.

The considered frame (tf2) for those values is tsl2561_link. Note that this node assumed that tf2 is installed.

This node is based on the motor_hat_node.cpp from matpalm (https://github.com/matpalm/ros-motorhat-node), the TSL2561.c file (https://github.com/ControlEverythingCommunity/TSL2561/tree/master/C) and the sensor documentation.

## wireless_controller.py
This node converts the input of a dual shock playstation controller to a twist message that can be used by the motor_hat_twist_node. Assuming that the controller is connected (using bluetooth network), this node sends a twist message named /wireless_controller/cmd_vel every time the joystick value changes.

Note that for now the physical address of the controller is hard-written in the code and you may want to change that...

Note that is uses evdev library to handle the bluetooth controller.

## The launch files 
In the istia_rover/launch folder there are several launch files.

### android_ros_control.launch
This launch file is used to launch to start the motor_hat_twist_node listening the /joy_teleop/cmd_vel topic. This launch file can be used when you want to control the robot with the ROS control android application.

### turtle_teleop.launch
This launch file is used to launch to start the motor_hat_twist_node listening the /turtle1/cmd_vel topic. This launch file can be used when you want to control the robot with the turtle teleop node.

### wireless_controller.launch
This launch file is used to launch to start the motor_hat_twist_node listening the /wireless_controller/cmd_vel topic. This is the launch file you want to use with the wireless_controller node (when using a dual shock controller).

### static_tf2_broadcast.launch
This launch file starts nodes that broacast static transforms for the LiDAR (/laser frame) and the lux sensor (/tsl2561_link frame).

# The script folder
This folder contains the scripts used, among others, to start the rover at boot. It is assumed that those scripts are in a folder name /home/pi/bin. All the logs are written in a folder named /home/pi/logs (you may want to create those folders first...)

## startup.sh
This is the script started at boot. In oder to be able to start the script at the boot of the raspberry pi, just add the line
```shell
$ sudo su -l pi -c "/home/pi/bin/startup.sh"
```
to your /etc/rc.local file. The "sudo su -l pi" is here to be shure that the script is executed as a pi user.

Note that this bash script assumed that you have tmux installed on the raspberry. If not, install it if you want to use this script
```shell
sudo apt-get install tmux
```
A tmux memo is given at the end of this readme file.

The startup.sh file is commented for further explanation. Note that it starts:
 - roscore
 - the listener.py script
 - urg_node (to handle the LiDAR, you may want to change that if you do not have the same LiDAR...)
 - tsl2561_node (to handle the lux sensor, you may want to change that if you do not have this sensor)
 - tf_broadcaster_node to handle the tf2 transforms of the sensors (LiDAR and lux sensor)
 - some tmux windows that are used by the listener.py script

## listener.py
This python script wait for the pad controller to be connected. Once it is connected, it starts the wireless_controller ros node and the motor_hat_twist_node using the wireless_controller.launch file.

Note that is uses evdev library to handle the bluetooth controller.

## TODO
Information about the other scripts

# Tmux memo
Those are the commands most often used when dealing with the istia rover

### from a classic terminal (not tmux)
kill tmux session:
```shell
tmux kill-session -t session_name
```
list tmux session:
```shell
tmux ls
```
attach tmux session:
```shell
tmux a -t session_name
```

### from tmux terminal
detach from tmux session (get back to classic terminal):
```shell
ctrl+b d
```
next window:
```shell
ctrl+b n
```
kill current window:
```shell
exit
```
# ROS memo
To install the contect of the workspace into the root of the ROS install (so you can remove the packages from your catkin_workspace, while use source code of standart library for instance)
```shell
 /opt/ros/kinetic/bin/catkin_make_isolated --install --install-space /opt/ros/kinetic
 ```
 This is the line while using ros kinetic installed in the /opt/ros/kinetic folder, this link sould be changed otherwise...
