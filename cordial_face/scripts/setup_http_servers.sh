#! /bin/bash

killall http-server

source /opt/ros/kinetic/setup.bash
source /home/qtrobot/catkin_ws/devel/setup.bash

roscd cordial_face/web && http-server -p $1 &
roscd cordial_tablet/web && http-server -p $2 &
