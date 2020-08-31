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
        - ```roslaunch harmoni_decision services.launch use_harmoni_services:=false```
        - ```roslaunch harmoni_decision routers.launch```
        - ```roslaunch harmoni_decision behavior_interface.launch test_service:="$REPO_$SERVER_$ID"```

    - From harmoni_pc:
        - ``` roslaunch harmoni_decision services.launch use_harmoni_services:=false ```

    - From ros_w2l:
        - ```roslaunch harmoni_stt stt_example.launch```

## Why Docker?
Although Harmoni will work without Docker, Harmoni is intended to be used with Docker to maximize portability and scalability. By using Docker Harmoni is quick and easy to set up on any OS or Hardware which currently supports docker.  We provide pre-built images for development, lightweight images for deployment, and even support for deploying to ARM chipsets.

## Docker Containers
We provide two sets of docker images. The first set is a heavier development set of images, which are based off of a ubuntu-16 image and contain graphical tools for use in a typical development cycle. These are intended to be helpful for developing interactively on a new machine. We also provide a set of lightweight images for deployment to space constrained machines, as well as images built with the arm architecture. 

All containers are built on Ros Kinetic with python 2.7, but with the catkin-workspace set up to default to python 3.6. In future releases we may build containers entirely without python2.7, including a ros installation that is built on python3.6. We may also choose to jump directly to Ros 2.

## Container Organization
Harmoni spreads the workload across several containers to maximize CPU usage. This includes a core Harmoni container, a container for interfacing with the hardware, and a container for each detector used.

The core Harmoni container is responsible for the following:

   - Central Control
   - Recording
   - IO with external services (e.g. cloud services)

The hardware container is responsible for the following:

   - Reading sensors
   - Controlling motors
   - Writing to display devices

The list of detector containers will expand over time, but currently includes:

   - Speech to Text
   - Face Detection


# Building 
## Harmoni

## Dev
```
docker build -f dockerfiles/dev/ubuntu16/dockerfile --tag cmbirmingham/ubuntu16-dev:latest .

docker build -f dockerfiles/dev/ros-kinetic/dockerfile --tag cmbirmingham/ros-kinetic-dev:latest .

docker build -f dockerfiles/dev/harmoni/dockerfile --tag cmbirmingham/harmoni-dev:latest .

docker build -f dockerfiles/dev/w2l/dockerfile --tag cmbirmingham/w2l-dev:latest .
```
## Lightweight
```
docker build -f dockerfiles/lightweight/ubuntu16/dockerfile --tag cmbirmingham/ubuntu16-lightweight:latest .

docker build -f dockerfiles/lightweight/ros-kinetic/dockerfile --tag cmbirmingham/ros-kinetic-lightweight:latest .

docker build -f dockerfiles/lightweight/harmoni/dockerfile --tag cmbirmingham/harmoni-lightweight:latest .

docker build -f dockerfiles/lightweight/w2l/dockerfile --tag cmbirmingham/w2l-lightweight:latest .
```

## ARM (Rasberri Pi)

[To build any of these images for ARM please start by following the instructions here](https://www.docker.com/blog/getting-started-with-docker-for-arm-on-linux/)

[Check also this link for buildx documentation](https://docs.docker.com/buildx/working-with-buildx/)

(Note: these are the instructions for building on an amd or intel device, to build a docker image on a Pi, just build like normal)
```
export DOCKER_CLI_EXPERIMENTAL=enabled

docker run --rm --privileged docker/binfmt:820fdd95a9972a5308930a2bdfb8573dd4447ad3 

docker buildx create --name mybuilder

docker buildx use mybuilder

docker buildx inspect --bootstrap


docker buildx build --push --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/arm/ubuntu16/dockerfile --tag cmbirmingham/ubuntu16-lightweight:arm .

docker buildx build --push --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/arm/ros-kinetic/dockerfile --tag cmbirmingham/ros-kinetic-lightweight:arm .

docker buildx build --push --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/arm/harmoni/dockerfile --tag cmbirmingham/harmoni-lightweight:arm .

docker buildx build --push --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/arm/w2l/dockerfile --tag cmbirmingham/w2l-lightweight:arm .
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