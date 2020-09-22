#!bin/bash

xhost +local:

export ROS_DISTRO="kinetic"

docker-compose up --remove-orphans
# docker-compose -f docker-compose-core.yml up
# docker-compose -f docker-compose-hardware.yml up

