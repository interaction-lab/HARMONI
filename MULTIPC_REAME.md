# How to Run with Multiple Computers

This walks through an approach of connecting multiple computers each running their own docker containers using host networks. There are a few other options with their own upsides/downsides that will be discussed in the Wiki.

## Running (QTRobot)

Run the following commands. A heading indicating #QTRP or #QTPC will be included each time context changes. Any host or container can be ROS Master as long as the ROS_MASTER_URI is set correctly.

__NOTE:__ It is assumed that the ROS_MASTER_URI and either ROS_HOSTNAME or ROS_IP environment variables are already set correctly on the host running the container. If you would like to run a container as ROS_MASTER, just ensure that the ROS_MASTER_URI matches the IP/hostname of the host (remember http and port is also included). 

__Commands:__

### docker run approach
```bash
#--QTPC--
#All paths must be absolute; double check the -v options to ensure they are correct. An assumed path was chosen for the harmoni git directory
#To run in background, replace "docker run -it" in first line with "docker run -dit" 
##copy until end
docker run -it --name harmoni_core --network host --restart=always --init \
--device=/dev/video0:/dev/video0 \
-e "ROS_MASTER_URI=$ROS_MASTER_URI" \
-e "ROS_HOSTNAME=$ROS_HOSTNAME" \
-e "ROS_IP=$ROS_IP" \
-e "ROS_DISTRO=kinetic" \
-e "CATKIN_WS=harmoni_catkin_ws" \
-e "IS_DOCKER_ENV=true" \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/setup_script.sh:"/root/.setup_script.sh" \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/dev_setup_script.sh:"/root/.dev_setup_script.sh" \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/asoundrc:"/root/.asoundrc" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/qtrobot/.gitconfig:/root/.gitconfig:ro \
-v /home/qtrobot/.ssh/:/root/.ssh:ro \
-v /home/qtrobot/.vimrc:/root/.vimrc:ro \
-v /home/qtrobot/.vim/:/root/.vim/:ro \
-v /etc/timezone:/etc/timezone:ro \
-v /etc/localtime:/etc/localtime:ro \
-w "/root/harmoni_catkin_ws/src/HARMONI" \
cmbirmingham/harmoni-lightweight:latest bash 
## end

#--QTRP--
#note that all paths must be absolute; double check the -v options to ensure they are correct.
#To run in background, replace "docker run -it" in first line with "docker run -dit" 
##copy until end
docker run -it --name harmoni_hardware --restart=always --network host --init \
--device=/dev/snd:/dev/snd \
-e "ROS_MASTER_URI=$ROS_MASTER_URI" \
-e "ROS_HOSTNAME=$ROS_HOSTNAME" \
-e "ROS_IP=$ROS_IP" \
-e "ROS_DISTRO=kinetic" \
-e "CATKIN_WS=harmoni_catkin_ws" \
-e "IS_DOCKER_ENV=true" \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/setup_script.sh:/root/.setup_script.sh \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/dev_setup_script.sh:/root/.dev_setup_script.sh \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/asoundrc:/root/.asoundrc \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/qtrobot/.gitconfig:/root/.gitconfig:ro \
-v /home/qtrobot/.ssh/:/root/.ssh:ro \
-v /home/qtrobot/.vimrc:/root/.vimrc:ro \
-v /home/qtrobot/.vim/:/root/.vim/:ro \
-v /etc/timezone:/etc/timezone:ro \
-v /etc/localtime:/etc/localtime:ro \
-w "/root/harmoni_catkin_ws/src/HARMONI" \
harmoniteam/lightweight:harmoni
## end

#communication verification (optional):
#--QTRP container--
rostopic list
#--QTPC host--
rostopic pub /test std_msgs/String "Hello"
#--QTRP container--
rostopic echo /test
```


## Troubleshooting
```bash
#get rid of old stuff if you have trouble starting again
docker container prune
docker network prune
docker swarm leave --force #only needed if there's some swarm issue (rare)

#get a terminal in the container when it's running in the background or doing something else.
docker exec -it <container_id> /bin/bash

docker logs -f <container_id>

# create swarm's overlay network from scratch (if using swarm)
#Be sure to only do this once on the manager. If it seems you have to do it twice, there is some other issue at play preventing network communication.
docker network create --driver=overlay --attachable --subnet 172.18.3.0/24 ros_net
```

If networking is still having trouble try running through [this tutorial](https://docs.docker.com/network/network-tutorial-overlay/#use-an-overlay-network-for-standalone-containers) and ensure that the example alpine containers are able to communicate.

# Discussion

TODO: update this and move to Wiki -- points below are old and missing some info/understanding

There are few different approaches for multi-host container networking we know about right now. They are as follows:

1. Attach to an external network (the approach chosen above). Good: flexible; Bad: docker-compose is finicky when it comes to swarm networking. I was unable to figure out how to get docker-compose to function on a second PC with an external network, while using the docker CLI worked just fine.
2. Create a docker stack/compose swarm (e.g. `docker stack deploy -c TEST-docker-compose-harmoni-swarm.yml <stack_name>`). Good: Can start all machines with a hands-off approach. Bad: Cannot specify static ips since this approach assumes you intend to make replicas of containers. This can worked around usually if you go by service name, which will generally provide IPs internally to containers in the same swarm.
3. Bridge containers to host network and make use of ROS communication methods. This theoretically would work, but it's unclear how you would establish a local IP in the container that could be contacted from another PC on the local network. The macvlan docker network option might work here as it creates a virtual network device. That said, this might not be very portable. Needs research/testing.
4. Use some other container management solution (Kubernetes, Overnode, etc.). These options might be great, but they require more research/testing.

*Important note:* Volumes are not shared across the network/swarms by default. Mount points are local to the machine they are run from, even when run from the swarm. There are alternate volume drivers from AWS that can do some more distributed data storage type things, but many people opt for a network drive if they want some shared file space.

Some sources:
https://docs.docker.com/network/network-tutorial-overlay/#use-an-overlay-network-for-standalone-containers
https://blog.alexellis.io/docker-stacks-attachable-networks/
https://docs.docker.com/compose/networking/#use-a-pre-existing-network
https://stackoverflow.com/questions/38088279/communication-between-multiple-docker-compose-projects
https://stackoverflow.com/questions/45180892/static-ip-address-doesnt-work-in-docker-compose-v3
https://stackoverflow.com/questions/47756029/how-does-docker-swarm-implement-volume-sharing



# Old approach (swarm with manual startup--didn't work for container<->pc communication)


Run the following commands. A heading indicating #manager or #worker will be included each time context changes. Either manager or worker can be the ROS Master/roscore. The current setup assumes the worker is the roscore/ROS master; if they are switched, then the set IPs in the docker compose and the docker run command below will need to change. 

__NOTE: There are two options for running everything. docker run or a partial usage of docker-compose and docker run. The docker run only option (#1) is more reliable while the docker-compose option is easier to do.__

For a QTRobot setup, it is assumed that the Worker & ROS master is the QTRP/RPI, while the docker manager is QTPC/NUC.

__Commands:__

```bash
#--manager--
#If your device has multiple interfaces, you may need to add the 
#"--advertise-addr <ip>" arg so that docker will know which network to use (it is usually the ip for eth0).
#If it's needed you'll get a message from docker.
docker swarm init
#copy join command that is displayed and run on worker next
#--worker--
docker swarm join --token <bunch of token info>
```

_Skip to option 1 or 2 depending on what you have chosen and then you should be done!_

### docker run only (Option 1)
```bash
#--manager--
docker network create --driver=overlay --attachable --subnet 172.18.3.0/24 ros_net

# verify ros_net is listed as a overlay swarm network (optional)
docker network ls # run in separate host terminal

#All paths must be absolute; double check the -v options to ensure they are correct. An assumed path was chosen for the harmoni git directory
#To run in background, replace "docker run -it" in first line with "docker run -dit" 
##copy until end
docker run -it --name harmoni_core --network ros_net --ip 172.18.3.5 --init \
--device=/dev/video0:/dev/video0 \
-e "ROS_MASTER_URI=http://172.18.3.4:11311" \
-e "ROS_HOSTNAME=harmoni_core" \
-e "ROS_DISTRO=kinetic" \
-e "CATKIN_WS=harmoni_catkin_ws" \
-e "IS_DOCKER_ENV=true" \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/setup_script.sh:"/root/.setup_script.sh" \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/dev_setup_script.sh:"/root/.dev_setup_script.sh" \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/asoundrc:"/root/.asoundrc" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/qtrobot/.gitconfig:/root/.gitconfig:ro \
-v /home/qtrobot/.ssh/:/root/.ssh:ro \
-v /home/qtrobot/.vimrc:/root/.vimrc:ro \
-v /home/qtrobot/.vim/:/root/.vim/:ro \
-v /etc/timezone:/etc/timezone:ro \
-v /etc/localtime:/etc/localtime:ro \
-p "11312:11312" \
-p "33691:33691" \
-p "8081:8081" \
-w "/root/harmoni_catkin_ws/src/HARMONI" \
cmbirmingham/harmoni-lightweight:latest bash 
## end

#--worker--
#note that all paths must be absolute; double check the -v options to ensure they are correct.
#To run in background, replace "docker run -it" in first line with "docker run -dit" 
##copy until end
docker run -it --name harmoni_hardware --network ros_net --ip 172.18.3.4 --init \
--device=/dev/snd:/dev/snd \
-e "ROS_MASTER_URI=http://172.18.3.4:11311" \
-e "ROS_HOSTNAME=harmoni_hardware" \
-e "ROS_DISTRO=kinetic" \
-e "CATKIN_WS=harmoni_catkin_ws" \
-e "IS_DOCKER_ENV=true" \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/setup_script.sh:/root/.setup_script.sh \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/dev_setup_script.sh:/root/.dev_setup_script.sh \
-v /home/qtrobot/harmoni_catkin_ws/src/HARMONI/dockerfiles/config/asoundrc:/root/.asoundrc \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/qtrobot/.gitconfig:/root/.gitconfig:ro \
-v /home/qtrobot/.ssh/:/root/.ssh:ro \
-v /home/qtrobot/.vimrc:/root/.vimrc:ro \
-v /home/qtrobot/.vim/:/root/.vim/:ro \
-v /etc/timezone:/etc/timezone:ro \
-v /etc/localtime:/etc/localtime:ro \
-p "11312:11312" \
-p "33691:33691" \
-p "8081:8081" \
-w "/root/harmoni_catkin_ws/src/HARMONI" \
harmoniteam/lightweight:harmoni bash 
## end

#--worker-container context--
roscore

#communication verification (optional):
#--worker container context--
rostopic list
#--master-container context--
rostopic pub /test std_msgs/String "Hello"
#--worker-container context--
rostopic echo /test
```