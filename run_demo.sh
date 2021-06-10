#!/bin/bash

roscore \
 & sleep 5 \
&& roslaunch harmoni_decision launcher.launch service:='harmoni' \
  & sleep 3 \
&& roslaunch harmoni_decision launcher.launch service:='hardware' \
  & sleep 3 \
&& roslaunch harmoni_pattern sequence_pattern.launch pattern_name:='demo' use_pattern_dialogue:=true use_pattern_multiple_choice:=false