# !/bin/bash

source /home/qtrobot/robot/autostart/qt_robot.inc

SCRIPT_NAME="start_audio_common.sh"
LOG_FILE=$(prepare_logfile "$SCRIPT_NAME")

{
prepare_ros_environment
wait_for_ros_node "/launch_audio_common" 60

roslaunch audio_capture capture_wave.launch

} &>> ${LOG_FILE}

