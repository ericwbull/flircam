#!/bin/bash

#source /opt/ros/kinetic/setup.bash
#source /home/pi/catkin_ws/devel/setup.bash
#cd /home/pi/gateway/logs
#rosbag record -a &
systemctl stop gateway
systemctl start gateway
roslaunch flircam gateway.launch

