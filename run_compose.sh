#!bin/bash

xhost +local:

export ROS_DISTRO="kinetic"

docker-compose up --remove-orphans
# docker-compose -f docker-compose-core.yml up
# docker-compose -f docker-compose-hardware.yml up

# Connect to containers by running the following in 
# a new terminal:
# docker exec -it [container_name] bash
# for example:
# docker exec -it harmoni_core bash
