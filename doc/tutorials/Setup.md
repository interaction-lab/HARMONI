# Setup


### Table of Contents  
[Setup with Docker (Recommended)](#setup-with-docker-recommended) \
[Setup harmoni_core without docker](#setup-harmoni_core-without-docker) \
[Configuration](#configuration)


# Get Setup and Running with Docker (Recommended)

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

3. **(optional)** Some Harmoni interactions require accounts with cloud services (Such as Amazon Polly). If you do not already have them set up, set up cloud services/keys:

    ### Setup Amazon Accounts
    (If you are planning to use Lex or Polly) Set up AWS account following these steps: 

    Create an Amazon Web Services account. AWS has a 1 year free trial that includes a limited number of Polly usages.
    Keep this in mind so you do not get charged money at the end of the year.

    Once you've created an account, create an IAM user to access Polly and/or Lex.

      * Give the IAM user access permissions to AWS Polly and/or Lex.
      * Give the IAM user access keys. Be sure to save the secret key as you only have one chance to look at it.
      
    https://docs.aws.amazon.com/cli/latest/userguide/install-linux.html - Use this link to install the AWS CLI on your PC.
    
    Then, in the terminal,
    ```bash
    $ sudo apt-get install awscli
    $ aws --version
    $ aws configure
    # Enter the IAM user access and secret keys here.
    ```

    ### Setup Google Accounts
    If you are planning on using dialogflow or other google services:
  
    You need to update private-keys.json in ./dockerfiles/config/ with your google credentials.
    When you activate google account API, you can create credentials for connecting HARMONI with your account, following this instraction.
    Get the API key (https://developers.google.com/maps/documentation/maps-static/get-api-key)
    You must have at least one API key associated with your project.

    To get an API key:

    * Go to the Google Cloud Platform Console.
    * Click the project drop-down and select or create the project for which you want to add an API key.
    * Click the menu button and select APIs & Services > Credentials.
    * On the Credentials page, click + Create Credentials > Create ID client OAuth.
    * Click Service Account, fill the input text with your name. Set the role to Editor, and click end.
    * Click the menu button and select APIs & Services > Credentials. In the Service Account table (at the bottom of the page) the account you just created will be displayed. Click on edit. Click on add new keys, and save it.
    * Click Close.
    * The new API key is listed on the Credentials page under API Keys.
    (Remember to restrict the API key before using it in production.)
    * Save private-keys.json

    Set credentials on HARMONI:

    ```bash
    $ cd ~/catkin_ws/src/HARMONI/dockerfiles/config/
    $ nano private-keys.json
    # Copy and paste the json content generated in the previous steps.
    ```

    Note: Secret keys and configurations are done locally and mounted to images through the Docker-Compose.yml files


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

# Setup harmoni_core without docker

Prerequisites: ROS Kinetic, Melodic, or Noetic Installed

1. Clone the repository:
   ```bash
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/interaction-lab/HARMONI.git
   cd ..
   ```

2. install dependencies:
   ```bash
   sudo apt-get install ros-$ROS_DISTRO-audio-common
   ```

3. Build harmoni core packages:
   ```bash
   catkin build harmoni_common_lib harmoni_common_msgs harmoni_decision harmoni_pattern harmoni_recorder
   ```
   If you do not have catkin tools installed, follow this instruction (https://catkin-tools.readthedocs.io/en/latest/installing.html) or also install from pip running:
   
   ```bash
   pip3 install --user git+https://github.com/catkin/catkin_tools.git
   ```
## TODO
For now this is a WIP. We recommend installing with python3 from source, like [here](https://www.miguelalonsojr.com/blog/robotics/ros/python3/2019/08/20/ros-melodic-python-3-build.html)

We recommend extending the blacklist packages to include those shown in the following sample command:
```bash
catkin config --init -DCMAKE_BUILD_TYPE=Release --blacklist rqt_rviz rviz_plugin_tutorials librviz_tutorial turtlesim cv_bridge rqt_image_view turtle_actionlib turtle_tf2 turtle_tf rviz rviz_python_tutorial python_orocos_kdl --install
```

We provide the script installation.sh to set up after ROS is installed.

# Configuration

*How to configure hardware and external endpoints that connect to HARMONI.*

## Audio

Check the `.asoundrc` file (located in `HARMONI/dockerfiles/config`) to ensure it refers to the appropriate sound device on your system. Determine which audio device you want to use by running `aplay -l` or `arecord -l` and update aforementioned `.asoundrc` file. *Note: make sure you can see hidden files*.

## Video

Usually the defaults will work here. If you have trouble, verify which device you are pulling video from ([reference](https://askubuntu.com/a/848390)): 
```bash
sudo apt-get install v4l-utils
v4l2-ctl --list-devices
```
Then modify your docker-compose file (e.g. `docker-compose-full.yml`) to select the desired video device. If you wanted to use `/dev/video1`, then you would replace the line under `devices:` which says `/dev/video0:/dev/video0` with `/dev/video1:/dev/video`

## Cloud Services

This has mostly been covered. In general, ensure you have stored your private keys/config/credentials in `~/.aws` or `~/.gcp`. The associated command line tools will usually do this for you.