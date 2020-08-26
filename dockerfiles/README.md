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

## Dev
```
docker build -f dockerfiles/dev/ubuntu16/dockerfile --tag cmbirmingham/ubuntu16-dev:latest .

docker build -f dockerfiles/dev/ros-kinetic/dockerfile --tag cmbirmingham/ros-kinetic-dev:latest .

docker build -f dockerfiles/dev/harmoni/dockerfile --tag cmbirmingham/harmoni-dev:latest .

docker build -f dockerfiles/dev/harmoni-pc/dockerfile --tag cmbirmingham/harmoni-pc-dev:latest .

docker build -f dockerfiles/dev/w2l/dockerfile --tag cmbirmingham/w2l-dev:latest .
```
## Lightweight
```
docker build -f dockerfiles/lightweight/ubuntu16/dockerfile --tag cmbirmingham/ubuntu16-lightweight:latest .

docker build -f dockerfiles/lightweight/ros-kinetic/dockerfile --tag cmbirmingham/ros-kinetic-lightweight:latest .

docker build -f dockerfiles/lightweight/harmoni/dockerfile --tag cmbirmingham/harmoni-lightweight:latest .

docker build -f dockerfiles/lightweight/harmoni-pc/dockerfile --tag cmbirmingham/harmoni-pc-lightweight:latest .

docker build -f dockerfiles/lightweight/w2l/dockerfile --tag cmbirmingham/w2l-lightweight:latest .
```

## ARM (Rasberri Pi)

[To build any of these images for ARM please start by following the instructions here](https://www.docker.com/blog/getting-started-with-docker-for-arm-on-linux/)]
[Check also this link for buildx documentation](https://docs.docker.com/buildx/working-with-buildx/)
```
export DOCKER_CLI_EXPERIMENTAL=enabled

docker buildx create

docker buildx use <name>


docker buildx build --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/lightweight/ubuntu16/dockerfile --tag cmbirmingham/ubuntu16-lightweight:arm .

docker buildx build --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/lightweight/ros-kinetic/dockerfile --tag cmbirmingham/ros-kinetic-lightweight:arm .

docker buildx build --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/lightweight/harmoni/dockerfile --tag cmbirmingham/harmoni-lightweight:arm .

docker buildx build --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/lightweight/harmoni-pc/dockerfile --tag cmbirmingham/harmoni-pc-lightweight:arm .

docker buildx build --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/lightweight/w2l/dockerfile --tag cmbirmingham/w2l-lightweight:arm .
```

# Network Notes
docker run --net mynet123 -h myhostname --ip 172.18.0.22 -it ubuntu bash

Compose example:
```
version: "2"
services:
  host1:
    networks:
      mynet:
        ipv4_address: 172.25.0.101
networks:
  mynet:
    driver: bridge
    ipam:
      config:
      - subnet: 172.25.0.0/24
```