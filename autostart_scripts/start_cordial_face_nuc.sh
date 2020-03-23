# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc

SCRIPT_NAME="start_cordial_face_nuc.sh"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/launch_face_server" 60

roslaunch cordial_face face_client.launch

} &>> ${LOG_FILE}

