#!/bin/bash

roslaunch harmoni_decision launcher.launch service:='harmoni' \
 & sleep 3 \
&& rosrun harmoni_web client_to_start_face.py \
  & sleep 3 \
&& roslaunch harmoni_decision harmoni_decision.launch