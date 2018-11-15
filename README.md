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

# The ROS istia_rover package
This package contains specifics nodes to run the rover and the sensors. Note that the source code of the nodes are in the istia_rover/src folder

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

The node publish three sensor_msgs::Illuminance messages (tsl2561/full_spectrum, tsl2561/infrared, tsl2561/visible") at a 5Hz frequency.

The considered frame (tf2) for those values is tsl2561_link.

This node is based on the motor_hat_node from matpalm (https://github.com/matpalm/ros-motorhat-node), the TSL2561.c file (https://github.com/ControlEverythingCommunity/TSL2561/tree/master/C) and the sensor documentation.

## wireless_controller.py
This node converts the input of a dual shock playstation controller to a twist message that can be used by the motor_hat_twist_node. Assuming that the controller is connected (using bluetooth network), this node sends a twist message named /wireless_controller/cmd_vel every time the joystick value changes.

Note that for now the physicall address of the controller is hard-written in the code and you may want to change that...

## The launch files 


# The script folder




