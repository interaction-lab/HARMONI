# Harmoni and Docker

## What is Docker?
Docker is a tool for developing and running containers, which are virtual machines that runs in parallel with your OS. There is a lot that could be said about Docker, but you can read more [here](https://www.docker.com/).

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

## Getting started
Unless adding a new node which requires additional libraries that have not been included, it unlikely you will need to modify the dockerfiles. Once Docker has been installed, you should be able to quickly get up and running with docker-compose. Docker-compose files have been provided which launch core, hardware, and detector containers. You may need to modify the network, devices, or volumes to suit your hardware configuration.

Additional instructions for building and running images are provided in the [dockerfiles/README.md](https://github.com/interaction-lab/HARMONI/blob/develop/dockerfiles/README.md)
