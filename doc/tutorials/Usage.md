# Usage

### Table of Contents  
[Running HARMONI in Docker](#running-harmoni-in-docker) \
[Run HARMONI in Multiple Containers](#run-harmoni-in-multiple-containers) \



# Running HARMONI in Docker
### Starting up HARMONI with the harmoni_full container
1. Open the folder:
    ```bash
    cd ~/catkin_ws/src/HARMONI
    ```
2. **(Optional)** In order to run with window forwarding on linux use:
    ```bash
    xhost +local:
    ```
3. Use docker compose to launch the complete system (will build if necessary, use --build to force) when developing new containers:
    ```bash
    docker-compose -f docker-compose-full.yml up
    ```
4. For each new process open another terminal and in each of them run:
    ```bash
    docker exec -it harmoni_full bash
    ```
    *Note: if you wish to connect to other containers, simply replace harmoni_full with the other containers name, e.g. harmoni_hardware*  

If the terminal prints the following message "Hello! Welcome to Harmoni", you successfully enter the container.

### Running HARMONI within the container
1. Setup audio:  
    Check which is your default card sound running: `aplay -l`.  
    Then use the card information to edit the configuration file in the root.
    ```bash
    cd /root
    nano .asoundrc
    # Update the hardware and soundcard id.
    ```
    The contents are shown below:
    
    ```yml
    pcm.!default {
      type plug
      slave {
        pcm "hw:0,0"
      }
    }
    ctl.!default {
      type hw
       card 0
    }
    ```
     
    The speaker device is controlled by the hw values, currently listed as 0,0 (the card `x` should be the same number of the first number in `hw:"x,y"`, and `y` refers to the device number).  
    To choose a speaker you may need to use the command `aplay -l `  to see what devices are available.  
    For example running `aplay -l`:
    ```
    card 1: PCH [HDA Intel PCH], device 0: ALC283 Analog [ALC283 Analog]
    ```
    In this case the `.asoundrc` file will be:
    ```yml
    pcm.!default {
      type plug
      slave {
        pcm "hw:1,0"
      }
    }
    ctl.!default {
      type hw
       card 1
    }
    ```    
    
2. Test the audio with the following command:
   ```bash
   roslaunch harmoni_speaker speaker_service test:=true
   ```
   If it works, exit and following the next steps.
   If it doesn't re-do step 1, and reiterate step 2 until the audio works.

3. Run the following bash script:

    ```bash
    run_demo.sh
    ```
    The run_demo script contains different commands to start with HARMONI.
    To begin working with ROS, we started with `roscore`. The docker containers have been networked so we only need to invoke one core in one of the containers.  
    Then we startup the HARMONI modules (`rlharmoniservices`, `rlhardwareservices`), and we run the interaction demo with `rldemo`.


4. Open the following links in the browser:
    * http://172.18.3.4:8081 (face of the robot)
    * http://172.18.3.4:8082 (web interface for replying to the robot)

    If it does not display anything, please refresh the page.
    If it successfully works, a face will be display in the screen in http://172.18.3.4:8081, and a green screen will display in http://172.18.3.4:8082. 

5.  Reorganize the window in order to see face, web page, and the terminals

    You should listen to the robot greeting you (you will NOT see the lip-sync in the face browser in the demo). You can reply it typing your answer in http://172.18.3.4:8082. 
    When you type your name, the robot will reply you back. You can start a back and forth dialogue.

    If you cannot listen to the audio, please check again in the container the `/root/.asoundrc` file and edit it according to your sound card (see Step 1 of Run).

# Run HARMONI in Multiple Containers
### Starting up HARMONI with multiple containers
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

Four terminals (1 for the core, 1 for the platform, and 1 for each detector) will be open up:
1. harmoni_core (the ROS master)
2. ros_hardware (hardware services)
3. ros_w2l (Words-to-Letter for the speech-to-text detector)
4. harmoni_visual_detector (face detector)


### Run HARMONI with multiple containers
 
To begin working with ROS, we must start with roscore. The docker containers have been networked so we only need to invoke one core in one of the containers. In the harmoni_core terminal run:
```bash
roscore
```

We provide a set of alias shortcuts for launching your project quickly and easily. The full list is defined in dockerfiles/config/setup_script.sh. 


> In harmoni we seek to provide a single point of truth for the configuration of the routers and services. Rather than generating multiple launch files for each configuration set, we use the python api for roslaunch to dynamically launch files based on the current configuration. We use launcher.launch to start the launcher.py node, which will start up the routers and services listed in the configuration.yaml file.

#### Setting Up the Configuration
Before running the whole interaction, you need to setup the configuration.yaml file in the _harmoni_decision_ package.
It contains the list of the packages that you want to run and the corrispondent _id_. For example:
```
harmoni:
   tts: ["default"]
hardware:
   microphone: ["default"]
```

It means that the tts harmoni service will use the _default _configuration which is set in configuration.yaml of each single package (e.g., harmoni_tss/config/configuration.yaml). If you want to add a new configuration, you have to edit both the _id_ of the _harmoni_decision_ package and the configuration file of the service package. For example: 
- In the harmoni_decision/config/configuration.yaml:
```
harmoni:
   tts: ["ita"]
hardware:
   microphone: ["default"]
```
- In the harmoni_tts/config/configuration.yaml:
```
default_param: 
    region_name: "us-west-2"
    language: "en-US"
    outdir: "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data"
    wav_header_length: 24
    voice: "Ivy"
ita_param:
    region_name: "eu-west-2"
    language: "it-IT"
    outdir: "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data"
    wav_header_length: 24
    voice: "Giovanna"
```
The default harmoni_decision/config/configuration.yaml file is set to run the "multiple-choice" interaction. You can keep the configuration if you want to run it for testing. However, remember to set the configuration parameters of the services according to your platform hardwares or external service.

#### Launching Services
After launching the ROS master, you can also launch the other services. For instance, go to the ros_hardware terminal and launch all the hardware services with the following command:
```bash
rlhardwareservices
#alias rlhardwareservices="roslaunch harmoni_decision launcher.launch service:='hardware'"
```
After launching the ROS master, you can also launch the other detectors. For instance, go to the w2l terminal and launch speech-to-text detector with the following command:
```bash
rlspeech
#alias rlspeech="roslaunch harmoni_stt stt_service.launch"
```
or launch the face detector in the harmoni_visual_detector terminal
```bash
rlfacedetect
#alias rlfacedetect="roslaunch harmoni_face_detect face_detect_service.launch"
```

#### Running an Interaction
Now everything is setup for testing.
Once all the packages have been started up, you can run the interaction by running in the harmoni_core.
If you want to run the "multiple-choice" interaction, run:
```bash
rlmultiplechoice
#alias rlmultiplechoice="roslaunch harmoni_decision harmoni_decision.launch test:=true pattern_name:='multiple-choice'"
```

## Run Package
If you want to run a single package, you can directly launch the node from the command line.

### Run a Service
If it is an HARMONI service (except for detectors, which are run in their own containers), you can run it in the _harmoni_core_ container:
```bash
roslaunch harmoni_$PACKAGE $PACKAGE_service.launch
```
For example:
```bash
roslaunch harmoni_tts tts_service.launch
```
This command allows you to verify if the package is correctly setup. If you want to test it, look at the "Running a test" Section.
If you want to run a detector, use the same command but in the corresponding container. For example, for the stt package, use the ros_w2l container for running the script.
If it is an hardware service (except for detectors), you can run it in the _ros_hardware_ container:
```bash
roslaunch harmoni_$PACKAGE $PACKAGE_service.launch
```
For example:
```bash
roslaunch pc_microphone microphone_service.launch
```

Each launch file provides the user a set of arguments:
- test: bool (set it to true if you want to run a test, see Section "Running a test")
- test_input: string (set it according to the data that you want to input to the package)
- test_id: string (set it as the id you chose for the _configuration.yaml_ file setting in the harmoni_decision)

### Run a behavior pattern
If you want to run a behavior pattern and test it, in harmoni_core run:
```bash
roslaunch harmoni_pattern sequential_pattern.launch pattern_name:="$pattern_to_test"
```

For example, if you want to test the multiple_choice pattern run:
```bash
roslaunch harmoni_pattern sequential_pattern.launch pattern_name:="multiple-choice"
```

## Working with Cordial Face
We provide an implementation of a face from the [cordial package](https://github.com/ndennler/cordial-public) implemented by the Interaction Lab. Although it is started as one face, we have implemented separate services for the eyes and mouth, allowing them to be controlled independently.

[[images/screen_demo.png]]