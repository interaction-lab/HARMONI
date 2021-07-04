# Multiple PCs

This walks through an approach of connecting multiple computers each running their own docker containers using host networks. There are a few other options with their own pro/cons found under [Discussion](#discussion).

## Host Network Approach

_This provided example covers the two PC setup that exists on QTrobot, which was an early deployment platform._

Run the following commands. A heading indicating #QTRP or #QTPC will be included each time context changes. Any host or container can be ROS Master as long as the ROS_MASTER_URI is set correctly.

__NOTE:__ It is assumed that the ROS_MASTER_URI and either ROS_HOSTNAME or ROS_IP environment variables are already set correctly on the host running the container. If you would like to run a container as ROS_MASTER, just ensure that the ROS_MASTER_URI matches the IP/hostname of the host (remember http and port is also included). 

__Commands:__

### docker run approach
```bash
#--QTPC--
#All paths must be absolute; double check the -v options to ensure they are correct. An assumed path was chosen for the harmoni git directory
#To run in background, replace "docker run -it" in first line with "docker run -dit" 
##copy until end
docker run -it --name harmoni_core --network host --init \
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
docker run -it --name harmoni_hardware --network host --init \
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

## Swarm Network Approach 

_Swarm with manual startup: note that it didn't work for container<->pc communication_

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

### Troubleshooting
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


## Discussion

There are few different approaches for multi-host container networking we know about right now. These approaches may change depending on where ROS Master/roscore is running from. They are as follows:

### **Bridge Networks (Nope!)**

If containers are being run on multiple hosts, they won't be on the same bridge network and thus will not be able to communicate directly. This could maybe be worked around with clever port forwarding and ROS_IPs in each container matching the local IP of their host to enable ROS communication, but it's not worth it.

### **Host Networks (Works!)**

This works as shown in the example above. The main thing needed to make it work in containers is to inherit the following environment variables from the host: ROS_MASTER_URI, ROS_HOSTNAME, ROS_IP. As long as hosts are able to talk to each other on the LAN, the containers should also be able to.

### Overlay Networks / Swarm / Services (Works, but...)

Usage of swarm networking requires that ROS Master be located inside of one of the swarm containers (cannot be located on the host PC unless you want to do quite a bit of setup). An example for this approach is shown under [Swarm approach](#swarm-approach).

The primary issue with using this solution with a ROS Master which is running outside of the swarm is that [ROS allocates random ports](https://answers.ros.org/question/198150/which-ports-are-needed-for-ros/?answer=198154#post-id-198154) provided by the OS, so port mapping will not easily work. There [are ways to port forward everything](http://wiki.ros.org/ROS/Tutorials/MultipleRemoteMachines#PortForwarding_.28PF.29), but it's probably not worth the effort. If someone does try this, it seems ROS can use any port from 1024-65550 (ports 0-1023 are reserved for specific system stuff). `docker run` can expose port ranges (e.g. `docker run -p 1024-65000:1024-65000 alpine`) but [there might be some bugs if large port ranges are exposed](https://github.com/moby/moby/issues/11185). In addition, Static IPs are not officially supported. In testing we were able to force static IPs by specifying the subnet in the network driver and specifying the IP in docker-compose or docker run, but not sure we should rely on that. If you use more than one replica of a service it could cause issues. 

Another approach which might work is to make [hostnames follow a template](https://docs.docker.com/engine/swarm/services/#create-services-using-templates), so you should be able to set it up so the hostname (and thus the ROS_HOSTNAME) is always known [and then have the container hostname exposed to the host (dns proxy server)](https://stackoverflow.com/questions/37242217/access-docker-container-from-host-using-containers-name/45071126#45071126). The method would work for the local host without port forwarding, but it's unclear if it would work for both hosts (needs testing—not sure if a container is accessible to the entire LAN or just the local host with this dns proxy).

### Macvlan **(Probably Works—Untested)**

This is similar to the host networking option, except instead of sharing the host network/IP, each docker container will get its own IP. This means the ROS IP of each container will just be the container's set IP. This option is good for using static IPs that are visible to the host and other hosts/containers that have access to the same LAN.

### Hybrid (Why?)

It's possible to use multiple networks on the same container. It's unclear how ROS_IP or ROS_HOSTNAME would be set to satisfy all networks though. It's also unclear why you would want this given some other options will work by themselves.

### Something Else (?)

Docker is constantly getting new features and options as time goes on. Some of the newer ones are not well documented but may serve our use case perfectly. I also did not research other network drivers/plugins which have been developed by 3rd parties. Container orchestration systems (Kubernetes, Openshift, etc.) other than docker swarm may also work well.


__Some sources:__  
[Use an overlay network for standalone containers](https://docs.docker.com/network/network-tutorial-overlay/#use-an-overlay-network-for-standalone-containers)  
[Docker stacks attachable networks](https://blog.alexellis.io/docker-stacks-attachable-networks/)  
[Use a pre-existing network](https://docs.docker.com/compose/networking/#use-a-pre-existing-network)  
[Communication between multiple docker compose projects](https://stackoverflow.com/questions/38088279/communication-between-multiple-docker-compose-projects)  
[Static ip address doesnt work in docker compose v3](https://stackoverflow.com/questions/45180892/static-ip-address-doesnt-work-in-docker-compose-v3)  
[How does docker swarm implement volume sharing?](https://stackoverflow.com/questions/47756029/how-does-docker-swarm-implement-volume-sharing)  
