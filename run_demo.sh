#!/bin/bash
  roscore \
& roslaunch harmoni_decision launcher.launch service:='harmoni' \
& roslaunch harmoni_decision launcher.launch service:='hardware' \
& roslaunch harmoni_pattern sequence_pattern.launch pattern_name:='demo' test:=true use_pattern_dialogue:=true use_pattern_multiple_choice:=false