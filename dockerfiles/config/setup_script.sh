#!/bin/bash

echo 'Hello, rosmaster'

source /opt/ros/$ROS_DISTRO/setup.bash 
source /root/harmoni_catkin_ws/devel/setup.bash

alias rh="roscd; cd .."
alias rs="roscd; cd ../src"
alias cm="roscd; cd ..; catkin_make"
alias cb="roscd; cd ..; catkin build"
alias cbs="roscd; cd ..; catkin build; source devel/setup.bash"
alias sd="roscd; cd ..; source devel/setup.bash"
alias rlspeech="roslaunch harmoni_stt stt_service.launch"
alias rlpcservices="roslaunch harmoni_decision launcher.launch router:=false service:='pc'"
alias rlharmoniservices="roslaunch harmoni_decision launcher.launch router:=false service:='harmoni'"
alias rlharmoni="roslaunch harmoni_decision launcher.launch service:='harmoni'"
alias rlrouter="roslaunch harmoni_decision launcher.launch service:=''"
alias rlbehavior="roslaunch harmoni_decision behavior_interface.launch"
