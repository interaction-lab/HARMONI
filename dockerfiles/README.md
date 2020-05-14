# Using Harmoni with Docker

To launch the complete harmoni dev setup in docker:
1. In order to run with window forwarding on linux use:
```
    xhost +local:
```

2. Use docker compose to launch the complete system (will build if necessary, use --build to force)
```
    docker-compose -f docker-compose-harmoni-dev.yml up
```

3. Launch the desired packages and run the desired scripts in the respective docker containers

    For example, to use Wave2Letter (requires running get_w2l_models.sh first):
    - From harmoni_core, open two terminals and:
        - ```roscore```
        - ```roslaunch harmoni_decision services.launch use_pc_services:=false```
        - ```roslaunch harmoni_decision routers.launch```
        - ```roslaunch harmoni_decision behavior_interface.launch test_service:="$REPO_$SERVER_$ID"```

    - From harmoni_pc:
        - ``` roslaunch harmoni_decision services.launch use_harmoni_services:=false ```

    - From ros_w2l:
        - ```roslaunch harmoni_stt stt_example.launch```


# Still todo:




# Docker and Harmoni Organization

The center of the Harmoni tool runs on a single core container, which is responsible for the following:
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

Around that core exists an ecosystem of supporting children, which will live in their own containers and interface with hardware or with custom processes. These children are coupled with the harmoni core libaries to provide standard communication and control methods, making them easy to extend or replace. 
- Harmoni Dialogue (1 Container Active)

- Harmoni Detectors (N Containers)

- Harmoni [External] (1 Container in individual repo, e.g. Harmoni-PC)




# Docker Containers

We provide two sets of docker containers. The first set is a heavier development set of containers, which are based off of a ubuntu-16 image and contain graphical tools for use in a typical development cycle. These containers are built on top of one another and are named with their development purpose and the suffix -dev.

The second set of containers are for use in deployment or on machines with limited space. These are built on top of the OSRF distribution of ros-kinetic, and have been named according to what was included in the container.

All containers are built on Ros Kinetic with python 2.7, but with the catkin-workspace set up to default to python 3.6. In future releases we may build containers entirely without python2.7, including a ros installation that is built on python3.6. We may also choose to jump directly to Ros 2.



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
