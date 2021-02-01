#!/usr/bin/env bash
set -e

echo 'Hello, rosmaster'

source /opt/ros/$ROS_DISTRO/setup.bash 
source /root/harmoni_catkin_ws/devel/setup.bash
bash run_qt_pi.sh

exec "$@"