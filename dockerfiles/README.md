# Using Harmoni with Docker

To launch the complete harmoni setup in docker:
1. In order to run with window forwarding on linux use:\
    xhost +local:

2. Use docker compose to launch the complete system (will build if necessary, use --build to force)\
    docker-compose -f docker-compose-ros-dev.yml up

3. Build the workspace \
    catkin build (in catkin_workspace)\
    source devel/setup.bash

4. Launch the desired packages and run the desired scripts in the respective docker containers

    For use with with Wave2Letter:
    - From the ros_dev:
        - roslaunch harmoni_stt microphone_service.launch
    - From w2l_ros:
        - python3.6 harmoni_stt/src/py_test_inference.py


# Still todo:

[] single launch bash script \
[] separate the rest of the continers (below)\
[] modularize containers and organize dependencies\
[] update readme\
[] setup cloning harmoni repo and building
[] setup cloning harmoni-[external] repo and building

# Docker and Harmoni Organization

Harmoni will be broken into several components:
- Harmoni Core includes:
    - Central Control
        - Harmoni Decision Manager
        - Harmoni Knowledge Store (Possibly seperate out?)
        - Harmoni Behavior Patterns
    - Common
        - Harmoni Common Lib
        - Harmoni Common Messages
    - Routers
        - Dialogue
        - Detectors
        - Actuators
        - Sensors
        - Web
The children of the routers will each be containers
- Harmoni Dialogue (1 Container Active)

- Harmoni Detectors (N Containers)

- Harmoni [External] (1 Container in individual repo, e.g. Harmoni-PC)
