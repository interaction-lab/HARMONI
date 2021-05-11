# Run HARMONI in Multiple Containers
### Startup
Open the folder:
```bash
cd ~/catkin_ws/src/HARMONI
```
**(Optional)** In order to run with window forwarding on linux use:
```bash
xhost +local:
```
 Use docker compose to launch the complete system (will build if necessary, use --build to force) for developing:
```bash
docker-compose -f docker-compose-harmoni-dev.yml up
```

If the terminal prints: "Hello, rosmaster", you successfully setup the HARMONI full container.  
*Note: We provide a bash script for launching multiple containers called run_compose.sh*

To connect your terminal to a running container execute:
```bash
docker exec -it $CONTAINER_NAME bash
```

To connect to each of the containers you should connect with four seperate terminals:
1. harmoni_core (the ROS master)
2. ros_hardware (hardware services)
3. ros_w2l (Words-to-Letter for the speech-to-text detector)
4. harmoni_visual_detector (face detector)


### Executing Across Containers
 
To begin working with ROS, we must start with roscore. The docker containers have been networked so we only need to invoke one core in one of the containers. In the terminal you have connected to harmoni_core run:
```bash
roscore
```

You can now launch any service in any container and they will communicate with each other. See 'Launching Services' for an example.

