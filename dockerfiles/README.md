# Using Harmoni with Docker

**Important** Please note that the kinetic build is not currently functional and the noetic build uses a kinetic w2l image because the w2l is not supported on 20.04. 

To launch the complete harmoni dev setup in docker:
1. In order to run on linux use:
```bash
    sh run_compose.sh
```

2. Launch the desired packages and run the desired scripts in the respective docker containers

    For example, to use Wave2Letter:
    - From harmoni_full, open a terminal and:
        - ```roscore```

    - From harmoni_hardware:
        - ``` roslaunch harmoni_microphone microphone_service.launch test:=true```
        - _Note: you can also use the alias ```rlhardwareservices``` we have provided, which will launch the hardware you have set in the configuration_

    - From harmoni_w2l:
        - ```roslaunch harmoni_stt stt_example.launch```
         - _Note: you can also use the alias ```rlspeech``` we have provided_

## Why Docker?
Although Harmoni will work without Docker, Harmoni is intended to be used with Docker to maximize portability and scalability. By using Docker Harmoni is quick and easy to set up on any OS or Hardware which currently supports docker.  We provide pre-built images for development, lightweight images for deployment, and even support for deploying to ARM chipsets.

Docker also allows us to (mostly) mix and match OS/ROS versions in different containers. For instance we use noetic with vision containers because of the python3-opencv integration but we use kinetic with w2l because facebook doesn't support w2l on 20.04 yet. This works for communication with standard messages and harmoni messages but may cause issues with other message types. [See here for more information.](https://answers.ros.org/question/45153/ros-fuerte-node-communicating-with-ros-electric-node/?answer=45160#post-id-45160)

## Docker Containers
We currently support development on ROS1 Noetic (passing) and Kinetic (broken - fix incoming). We also provide a development container with additional tools and ML libraries.

## Container Organization
Harmoni spreads the workload across several containers to maximize CPU usage. This includes a full Harmoni container, a container for interfacing with the hardware, and a container for each detector used.

The full Harmoni container is responsible for the following:

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


# Building Harmoni

## Noetic
```bash
docker build -f dockerfiles/harmoni/noetic/base/dockerfile --tag harmoniteam/harmoni:noetic-base .

docker build -f dockerfiles/harmoni/noetic/full/dockerfile --tag harmoniteam/harmoni:noetic-full .

# W2L is not currently supported on Ubuntu 20.04
# docker build -f dockerfiles/harmoni/noetic/w2l/dockerfile --tag harmoniteam/harmoni:noetic-w2l .

docker build -f dockerfiles/harmoni/noetic/facedetect/dockerfile --tag harmoniteam/harmoni:noetic-facedetect .
```

## Kinetic
```bash
docker build -f dockerfiles/harmoni/kinetic/base/dockerfile --tag harmoniteam/harmoni:kinetic-base .

docker build -f dockerfiles/harmoni/kinetic/full/dockerfile --tag harmoniteam/harmoni:kinetic-full .

docker build -f dockerfiles/harmoni/kinetic/w2l/dockerfile --tag harmoniteam/harmoni:kinetic-w2l .

docker build -f dockerfiles/harmoni/kinetic/face_detect/dockerfile --tag harmoniteam/harmoni:kinetic-face_detect .
```

## Dev
```bash
docker build -f dockerfiles/harmoni-dev/full/dockerfile --tag harmoniteam/harmoni-dev:kinetic-harmoni .
```

## MultiArchitecture (ARM/Rasberry Pi)

[To build any of these images for ARM please start by following the instructions here](https://www.docker.com/blog/getting-started-with-docker-for-arm-on-linux/)

[Check also this link for buildx documentation](https://docs.docker.com/buildx/working-with-buildx/)

[Also useful: automate builds on github](https://github.com/marketplace/actions/docker-buildx)

(Note: these are the instructions for building on an amd or intel device, to build a docker image on a Pi, just build like normal)
```bash
export DOCKER_CLI_EXPERIMENTAL=enabled

docker run --privileged --rm tonistiigi/binfmt --install all
docker buildx create --name mybuilder
docker buildx use mybuilder
docker buildx inspect --bootstrap

docker buildx build --cache-from "type=local,src=/tmp/.buildx-cache" --cache-to "type=local,dest=/tmp/.buildx-cache" --output "type=image,push=true" --platform linux/amd64,linux/arm64,linux/arm/v7 -f dockerfiles/harmoni/noetic/base/dockerfile --tag harmoniteam/harmoni:noetic-base .

```

Running and connecting to the terminal
```bash
docker exec -it <containername> bash 
```

# Network Notes
docker run --net mynet123 -h myhostname --ip 172.18.0.22 -it ubuntu bash

Compose example:
```docker
version: "2"
services:
  host1:
    networks:
      mynet:
        ipv4_address: 172.25.0.101
...
...
networks:
  mynet:
    driver: bridge
    ipam:
      config:
      - subnet: 172.25.0.0/24
```
