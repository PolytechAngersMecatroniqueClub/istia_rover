# istia_rover
This repository contains the ROS nodes used on the rover and some usefull scripts an explaination

# the material parts of the rover
the rover is based on:
 - a 4 Wheels Scout Platform Robot Kit from ServoCity (https://www.servocity.com/scout)
 - a raspberry pi 3B+
 - a motor hat stepper motor shield from adafruit (https://www.adafruit.com/product/2348)
 - two alimentations boards UNI-REG (https://www.lextronic.fr/conversion-tension/29859-module-alimentation-1-8-2-7-3-3-4-5-12vcc.html)
 - a LiDAR hokuyo utm-30lx (with an home made power board for power supply the sensor)
 - a TSL2561 lux sensor from adafruit (https://www.adafruit.com/product/439)

# The ROS istia_rover package
This package contains specifics nodes to run the rover dans the sensors, the source code of the nodes are in the src folder

## motor_hat_twist_node.cpp
This node allows to convert a twist message to pwm commands for the motors. It is based on the motor_hat_node from matpalm (https://github.com/matpalm/ros-motorhat-node)



## tsl2561_node.cpp

## wireless_controller.py

## The launch files



