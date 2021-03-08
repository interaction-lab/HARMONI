#!/bin/bash

roslaunch harmoni_speaker speaker_play.launch \
  & sleep 3 \
&& rosrun harmoni_web server_to_start_face.py


