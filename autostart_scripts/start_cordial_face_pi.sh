# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc

SCRIPT_NAME="start_cordial_face_pi.sh"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/DB1/face" 60
wait_for_tcpip_port 1883 60

roslaunch cordial_face server_for_browser_on_pi.launch

} &>> ${LOG_FILE}

