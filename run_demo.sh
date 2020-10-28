#!/bin/bash

roscore \
 & sleep 5 \
&& roslaunch harmoni_decision launcher.launch service:='harmoni' \
  & sleep 3 \
&& roslaunch harmoni_decision launcher.launch service:='hardware' \
  & sleep 3 \
&& roslaunch harmoni_pattern sequence_pattern.launch pattern_name:='demo' test:=true