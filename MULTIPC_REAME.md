# How to Run with Multiple Computers

This walks through an approach of connecting multiple computers each running their own docker containers using network extension. There are a few other options with their own upsides/downsides discussed at the end.

## Running

Run the following commands. A heading indicating #manager or #worker will be included each time context changes. Either manager or worker can be the ROS Master/roscore. The current setup assumes the worker is the roscore/ROS master; if they are switched, then the set IPs in the docker compose and the docker run command below will need to change. 

For a QTRobot setup, it is assumed that the Worker & ROS master is the QTRP/RPI, while the docker manager is QTPC/NUC

```bash
#--manager--
docker swarm init
#copy join command that is displayed and run on worker next
#--worker--
docker swarm join --token <bunch of token info>

#--manager--
docker-compose -f docker-compose-harmoni-lightweight-multipc.yml up

# verify ros_net is listed as a overlay swarm network (optional)
docker network ls # run in separate host terminal

#--worker--
#this is one command
docker run -it --name hardware --network ros_net --ip 172.18.3.4 --init \
--device=/dev/snd:/dev/snd \
-e ROS_MASTER_URI=http://172.18.3.4:11311 \
-e ROS_HOSTNAME=harmoni_hardware \
-e ROS_DISTRO=kinetic \
-e CATKIN_WS=harmoni_catkin_ws \
-e $DISPLAY -e QT_GRAPHICSSYSTEM=native -e IS_DOCKER_ENV="true" \
-v $(pwd)/dockerfiles/config/setup_script.sh:/root/.setup_script.sh \
-v $(pwd)/dockerfiles/config/dev_setup_script.sh:/root/.dev_setup_script.sh \
-v $(pwd)/dockerfiles/config/asoundrc:/root/.asoundrc \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ~/.gitconfig:/root/.gitconfig:ro \
-v ~/.ssh/:/root/.ssh:ro \
-v ~/.vimrc:/root/.vimrc:ro \
-v ~/.vim/:/root/.vim/:ro \
-v /etc/timezone:/etc/timezone:ro \
-v /etc/localtime:/etc/localtime:ro \
-p "11312:11312" \
-p "33691:33691" \
-p "8081:8081" \
-w /root/harmoni_catkin_ws/src/HARMONI \
harmoniteam/lightweight:harmoni bash 
#cmd end

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


## Troubleshooting
```bash
#get rid of old stuff if you have trouble starting again
docker container prune
docker network prune
docker swarm leave --force #only needed if there's some swarm issue (rare)

#get a terminal in the container when it's running in the background or doing something else.
docker exec -it <container_id> /bin/bash

docker logs -f <container_id>

#sometimes it's useful to create a network from scratch rather than relying on docker-compose to do it.
docker network create --driver=overlay --attachable --subnet 172.18.3.0/24 ros_net
```

# Discussion

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