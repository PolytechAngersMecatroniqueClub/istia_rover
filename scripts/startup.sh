#!/bin/bash
# this script is called "startup.sh"

LOG_STARTUP=/home/pi/logs/startup.log
LOG_TMUXSESS=/home/pi/logs/tmux_session.log
LOG_ROSCORE=/home/pi/logs/tmux_roscore.log
LOG_LISTENER=/home/pi/logs/tmux_listener.log
LOG_CTRLNODE=/home/pi/logs/tmux_ctrlnode.log
LOG_MOTORHAT=/home/pi/logs/tmux_motorhat.log
LOG_LIDAR=/home/pi/logs/tmux_lidar.log
LOG_TSL2561=/home/pi/logs/tmux_tsl2561.log
LOG_TFBROADCASTER=/home/pi/logs/tmux_tfbroadcaster.log
LOG_ROSBAG=/home/pi/logs/tmux_rosbag.log

# if the startup.log file already exists, we make a backup just in case
# this is done in order not to overwrite the last logs when the raspberry pi reboots
if [ -f $LOG_STARTUP ]
then
    mv $LOG_STARTUP $LOG_STARTUP.back
fi
# starting the startup.log file
# the command done in this startup.sh file will the redirected to this log file
echo "startup.log file" > $LOG_STARTUP
date +"%Y %m %d %H %M %S" >> $LOG_STARTUP

# --- creation of the mux session ---

# if the tmux_session.log file already exists, we make a backup just in case
# this is done in order not to overwrite the last logs when the raspberry pi reboots
if [ -f $LOG_TMUXSESS ]
then
    mv $LOG_TMUXSESS $LOG_MUXSESS.back
fi
# starting the tmux_session.log file
# the command done in this startup.sh file will the redirected to this log file
echo "tmux_session.log file" > $LOG_TMUXSESS
date +"%Y %m %d %H %M %S" >> $LOG_TMUXSESS

echo "starting the ros tmux session" | tee -a $LOG_STARTUP

# starting the new tmux session
# -s : set the name of the session
# -d : start the session as a deamon
# the output of the command is redirected to the tee command (it allows to redirect the input both into a file and the terminal)
# the -a option for tee is to append the ouput at the end of the file
# the 2>&1 option before the pipe (|) command is to redirect both the standard and error ouputs
tmux new -s ros -d 2>&1 | tee -a $LOG_STARTUP
# pipe-pane option allows to redirect the ouput of the session/window
# -t : the name of the session to redirect
# in this case we redirect the ouput into the tee command, as before
# if everything goes well all the ouput of the ros tmux session should be written into the $LOG_TMUXSESS file
tmux pipe-pane -t ros "tee -a '$LOG_TMUXSESS'"

# --- creation of the roscore tmux window ---

# if the tmux_roscore.log file already exists, we make a backup just in case
# this is done in order not to overwrite the last logs when the raspberry pi reboots
if [ -f $LOG_ROSCORE ]
then
    mv $LOG_ROSCORE $LOG_ROSCORE.back
fi
# starting the tmux_session.log file
# the command done in this startup.sh file will the redirected to this log file
echo "tmux_roscore.log file" > $LOG_ROSCORE
date +"%Y %m %d %H %M %S" >> $LOG_ROSCORE

echo "starting the roscore tmux window" | tee -a $LOG_STARTUP

# starting the new window
# -d : new window as deamon
# -a : append the new window to the session
# -n : set the name of the new window
# -t : the name of the session to which the new window should be attached
# 2>&1 | : redirect all the error and standard ouputs of the command
# tee -a : append the ouput to the terminal and the log file
tmux new-window -d -a -n 'roscore' -t ros 2>&1 | tee -a $LOG_STARTUP
# pipe-pane option allows to redirect the ouput of the session/window
# -t : the name of the session/window to redirect
# in this case we redirect the ouput into the tee command, as before
# if everything goes well all the ouput of the roscore tmux window should be written into the $LOG_TMUXSESS file
tmux pipe-pane -t roscore "tee -a '$LOG_ROSCORE'"
# send a command to a specific window (roscore here)
# -t : the name of the window to send the command
# the command here is 'source ...'
# C-m : the 'enter' command to validate the command in the terminal
tmux send-keys -t 'roscore' 'source /home/pi/ros_catkin_ws/devel/setup.bash'  C-m
# start roscore in the terminal
tmux send-keys -t 'roscore' 'roscore' C-m

/home/pi/bin/ihm.py -c blue -t "starting roscore" | tee -a $LOG_STARTUP # display a message in the hmi

echo "sleeping 15s... waiting for roscore to be started ..." | tee -a $LOG_STARTUP

sleep 15 &
PID=$!
i=1
sp="/-\|"
echo -n ' '
while [ -d /proc/$PID ]
do
  printf "\b${sp:i++%${#sp}:1}"
done

echo "end of sleeping" | tee -a $LOG_STARTUP

/home/pi/bin/ihm.py -c green -t "roscore started"

# --- creation of the controller node tmux window ---
# check the previous comments for explanation

if [ -f $LOG_CTRLNODE ]
then
    mv $LOG_CTRLNODE $LOG_CTRLNODE.back
fi

echo "tmux_ctrlnode.log file" > $LOG_CTRLNODE
date +"%Y %m %d %H %M %S" >> $LOG_CTRLNODE

echo "starting the ctrlnode tmux window" >> $LOG_STARTUP

tmux new-window -d -a -n 'ctrlnode' -t ros 2>&1 | tee -a $LOG_STARTUP
tmux pipe-pane -t ctrlnode "tee -a '$LOG_CTRLNODE'"
tmux send-keys -t 'ctrlnode' 'source /home/pi/ros_catkin_ws/devel/setup.bash'  C-m
tmux send-keys -t 'ctrlnode' 'echo "waiting for the rosrun...."'  C-m


# --- creation of the motor hat node tmux window ---
# check the previous comments for explanation

if [ -f $LOG_MOTORHAT ]
then
    mv $LOG_MOTORHAT $LOG_MOTORHAT.back
fi

echo "tmux_motorhat.log file" > $LOG_MOTORHAT
date +"%Y %m %d %H %M %S" >> $LOG_MOTORHAT

echo "starting the motorhat tmux window" >> $LOG_STARTUP

tmux new-window -d -a -n 'motorhat' -t ros 2>&1 | tee -a $LOG_STARTUP
tmux pipe-pane -t motorhat "tee -a '$LOG_MOTORHAT'"
tmux send-keys -t 'motorhat' 'source /home/pi/ros_catkin_ws/devel/setup.bash'  C-m
tmux send-keys -t 'motorhat' 'export ROS_MASTER_URI=http://10.0.0.1:11311'  C-m
tmux send-keys -t 'motorhat' 'export ROS_IP=10.0.0.1'  C-m

tmux send-keys -t 'motorhat' 'echo "waiting for the roslaunch command"'  C-m

# --- creation of the wireless listener tmux window ---
# check the previous comments for explanation

if [ -f $LOG_LISTENER ]
then
    mv $LOG_LISTENER $LOG_LISTENER.back
fi

echo "tmux_listener.log file" > $LOG_LISTENER
date +"%Y %m %d %H %M %S" >> $LOG_LISTENER

echo "starting the listener tmux window" | tee -a $LOG_STARTUP

tmux new-window -d -a -n 'listener' -t ros 2>&1 | tee -a $LOG_STARTUP
tmux pipe-pane -t listener "tee -a '$LOG_LISTENER'"
tmux send-keys -t 'listener' '/home/pi/bin/listener.py'  C-m

# --- creation of the lidar ros node tmux window ---
# check the previous comments for explanation

if [ -f $LOG_LIDAR ]
then
    mv $LOG_LIDAR $LOG_LIDAR.back
fi

echo "tmux_lidar.log file" > $LOG_LIDAR
date +"%Y %m %d %H %M %S" >> $LOG_LIDAR

echo "starting the Lidar tmux window" | tee -a $LOG_STARTUP

tmux new-window -d -a -n 'lidar' -t ros 2>&1 | tee -a $LOG_STARTUP
tmux pipe-pane -t lidar "tee -a '$LOG_LIDAR'"
tmux send-keys -t 'lidar' 'rosrun urg_node urg_node'  C-m


# --- creation of the tsl2561 ros node tmux window ---
# check the previous comments for explanation

if [ -f $LOG_TSL2561 ]
then
    mv $LOG_TSL2561 $LOG_TSL2561.back
fi

echo "tmux_tsl2561.log file" > $LOG_TSL2561
date +"%Y %m %d %H %M %S" >> $LOG_TSL2561

echo "starting the tsl2561 tmux window" | tee -a $LOG_STARTUP

tmux new-window -d -a -n 'tsl2561' -t ros 2>&1 | tee -a $LOG_STARTUP
tmux pipe-pane -t tsl2561 "tee -a '$LOG_TSL2561'"
tmux send-keys -t 'tsl2561' 'rosrun istia_rover tsl2561_node'  C-m

# --- creation of the tfbroadcster ros node tmux window ---
# check the previous comments for explanation

if [ -f $LOG_TFBROADCASTER ]
then
    mv $LOG_TFBROADCASTER $LOG_TFBROADCASTER.back
fi

echo "tmux_tfbroadcaster.log file" > $LOG_TFBROADCASTER
date +"%Y %m %d %H %M %S" >> $LOG_TFBROADCASTER

echo "starting the tfbroadcaster tmux window" | tee -a $LOG_STARTUP

tmux new-window -d -a -n 'tfbroadcaster' -t ros 2>&1 | tee -a $LOG_STARTUP
tmux pipe-pane -t tfbroadcaster "tee -a '$LOG_TFBROADCASTER'"
tmux send-keys -t 'tfbroadcaster' 'roslaunch istia_rover static_tf2_broadcast.launch'  C-m
tmux send-keys -t 'tfbroadcaster' 'roslaunch istia_rover static_tf2_broadcast.launch'  C-m


# --- creation of the rosbag tmux window ---
# check the previous comments for explanation

if [ -f $LOG_ROSBAG ]
then
    mv $LOG_ROSBAG $LOG_ROSBAG.back
fi

echo "tmux_rosbag.log file" > $LOG_ROSBAG
date +"%Y %m %d %H %M %S" >> $LOG_ROSBAG

echo "starting the rosbag tmux window" | tee -a $LOG_STARTUP

# we get the current time into a variable for the name of the rosbag
NOW=`date '+%F_%H-%M-%S'`

tmux new-window -d -a -n 'rosbag' -t ros 2>&1 | tee -a $LOG_STARTUP
tmux pipe-pane -t rosbag "tee -a '$LOG_ROSBAG'"
tmux send-keys -t 'rosbag' 'echo "waiting for a rosbag to start..."'  C-m

exit 0
