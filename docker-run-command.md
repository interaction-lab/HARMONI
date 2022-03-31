
sudo docker run -it --name harmoni_core --network host --init \
-e "ROS_DISTRO=noetic" \
-e "CATKIN_WS=harmoni_catkin_ws" \
-e "IS_DOCKER_ENV=true" \
-v /home/micol/harmoni_fork_ws/src/HARMONI/dockerfiles/config/setup_script.sh:"/root/.setup_script.sh" \
-v /home/micol/harmoni_fork_ws/src/HARMONI/dockerfiles/config/dev_setup_script.sh:"/root/.dev_setup_script.sh" \
-v /home/micol/harmoni_fork_ws/src/HARMONI/dockerfiles/config/asoundrc:"/root/.asoundrc" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/micol/.ssh/:/root/.ssh:ro \
-v /home/micol/.vimrc:/root/.vimrc:ro \
-v /home/micol/.vim/:/root/.vim/:ro \
-v /etc/timezone:/etc/timezone:ro \
-v /etc/localtime:/etc/localtime:ro \
-v /home/micol/harmoni_fork_ws/src/HARMONI/:/root/local_mount/HARMONI/ \
-w "/root/harmoni_catkin_ws/src/HARMONI" \
harmoniteam/harmoni:noetic-full bash 