
# Quickstart With Docker

*Note: To complete these steps you will need to have docker working on your machine. We recommend you follow the steps here: https://docs.docker.com/engine/install/ubuntu/ and verify your install with:*
```bash
sudo docker run hello-world
```

To avoid entering sudo, run the following:
```bash
sudo usermod -aG docker ${USER}
su - ${USER}
```

## Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/interaction-lab/HARMONI.git
   cd HARMONI
   ```


2. **(optional)** Pull the containers:

    You may either build or pull containers to run harmoni. Pulling containers is generally faster and can be done as:

    ```bash
    docker pull harmoniteam/harmoni:kinetic-full
    ```

    *Note: If you would like to use all the containers we have provided a script in dockerfiles/pull_images.sh for getting all the current images, which includes both kinetic and noetic. If you run a compose file without pulling, the images will be automatically built.*

    For additional information on building or running harmoni containers, see dockerfiles/README.md.

3. **(optional)** Some Harmoni interactions require accounts with cloud services (Such as Amazon Polly). If you do not already have them set up, set up cloud services/keys. Instructions are in the Cloud Services section.


4. **(optional)** In order to run with window forwarding on linux use:
   ```bash
   xhost +local:
   ```

   Note: For Windows you will need to set up docker with WSL2


5. Use docker compose to launch the complete system **full** (will build if necessary, use --build to force):
   ```bash
   docker-compose -f docker-compose-full.yml up
   ``` 
   If the terminal prints: "Hello, rosmaster", you successfully setup the HARMONI full container.  
   *Note: We provide a bash script for launching multiple containers called run_compose.sh*

6. Open a new terminal. To get into this container, you will need to run the following to mount a terminal into the container:
   ```bash
   docker exec -it harmoni_full bash
   ``` 
   *Note: if you wish to connect to other containers, simply replace harmoni_full with the other containers name, e.g. harmoni_hardware*  

   If this is successful, you will see:
   ```bash
   Hello! Welcome to Harmoni
   11:47 AM [develop] ~/harmoni_catkin_ws/src/HARMONI :
   root@harmoni_full# 
   ``` 
   From this point you may launch `roscore` or any other ros or harmoni command. 
   *Note: You will need to do this step for every process you wish to run in the container. If multiple terminal processes are required, you will need to connect multiple terminals to the docker instance with the docker exec command above*  
   If the `roscore` run is successful, you will see:
   ```bash
   started core service [/rosout]
   ```

You are now set up for Harmoni with Docker! Go to Usage to see more about running Harmoni next.
