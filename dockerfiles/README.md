# Using Harmoni with Docker

To launch the complete harmoni setup in docker:
1. In order to run with window forwarding on linux use:
```
    xhost +local:
```

2. Use docker compose to launch the complete system (will build if necessary, use --build to force)
```
    docker-compose -f docker-compose-harmoni-dev.yml up
```
4. Launch the desired packages and run the desired scripts in the respective docker containers

    For use with with Wave2Letter:
    - From the ros_dev:
        - ```roslaunch harmoni_stt microphone_service.launch```
    - From w2l_ros:
        - ```python3.6 harmoni_stt/src/py_test_inference.py```


# Still todo:

- [X] make harmoni public
- [X] separate the rest of the continers (below)
- [X] modularize containers and organize dependencies
- [X] setup cloning harmoni repo and building
- [X] setup cloning harmoni-[external] repo and building
- [ ] single launch bash script 
- [ ] push images to dockerhub
- [ ] update readme



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




# Docker Containers


Base containers:
    osrf/ros:kinetic-desktop (ros_kinetic_base)
    ubuntu:xenial (ubuntu16_base)
Dev containers: base containers with extra tools
    ubuntu16_dev
    ros_kinetic_dev

Tier 1:
    ros_kinetic_harmoni
        # includes kinetic audio-common, numpy, pyaudio
        # built on python 3.6
        # with a built harmoni installed
    ros_kinetic_harmoni_w2l-inf
    ros_kinetic_harmoni_harmoni-pc


# Building 
## Harmoni
```
docker build -f dockerfiles/ros-kinetic_harmoni --tag harmoni/ros-kinetic_harmoni:latest --squash .

docker build -f dockerfiles/ros-kinetic_harmoni_harmoni-pc --tag harmoni/ros-kinetic_harmoni_harmoni-pc:latest --squash .

docker build -f dockerfiles/ros-kinetic_harmoni_w2l-inf --tag harmoni/ros-kinetic_harmoni_w2l-inf:latest --squash .
```
## Dev
```
docker build -f dockerfiles/dev/ubuntu16-dev --tag cmbirmingham/ubuntu16-dev:latest --squash .

docker build -f dockerfiles/dev/ros-kinetic-dev --tag cmbirmingham/ros-kinetic-dev:latest --squash .

docker build -f dockerfiles/dev/harmoni-dev --tag cmbirmingham/harmoni-dev:latest --squash .

docker build -f dockerfiles/dev/harmoni-pc-dev --tag cmbirmingham/harmoni-pc-dev:latest --squash .

docker build -f dockerfiles/dev/w2l-dev --tag cmbirmingham/w2l-dev:latest --squash .
```