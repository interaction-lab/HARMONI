#!/bin/bash

echo 'Hello, rosmaster'

source /opt/ros/$ROS_DISTRO/setup.bash 
source /root/harmoni_catkin_ws/devel/setup.bash

alias rh="roscd; cd .."
alias rs="roscd; cd ../src"
alias cm="roscd; cd ..; catkin_make"
alias cb="roscd; cd ..; catkin build"
