# Setting Up A Robot

STUB -- need to define interfaces that would be attached to

# HARMONI on QT 

For setting up HARMONI on QT, you need to handle a multi-pc architecture. QT has a PI on its head, and a NUC in the chest.  

The PI is connected with the: 

- Speaker 
- Microphone 
- Screen of the face 

The NUC will contain the main logic of the whole activity and the harmoni_core services.  

In the following sections, all the steps for setting up docker and installing HARMONI are reported again for working with QT. 

## Docker Setup (to do on both NUC and PI) 

 

To complete these steps you will need to have docker working on your machine. We recommend you follow the steps here: [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/) and verify your install with: 

```bash 
sudo docker run hello-world 
``` 

 
To avoid entering sudo, run the following: 

```bash 
sudo usermod -aG docker ${USER} 
su - ${USER} 
``` 

You will also need to install docker-compose. Instructions can be found here: [https://docs.docker.com/compose/install/](https://docs.docker.com/compose/install/) 

 

## HARMONI Install (to do on both NUC and PI) 

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
 

3. **(optional)** Some Harmoni interactions require accounts with cloud services (Such as Amazon Polly). If you do not already have them set up, set up cloud services/keys. Instructions are in the [Cloud Services section](../configuration/Cloud-Services). 



## Run HARMONI on the NUC 


1. **(optional)** In order to run with window forwarding on linux use: 

   ```bash 
   xhost +local: 
   ``` 

 

   Note: For Windows you will need to set up docker with WSL2 

 

 

2. Use docker compose to launch the complete system (will build if necessary, use --build to force): 

   ```bash 
   docker-compose -f docker-compose-qt-nuc.yml up 
   ```  

   If the terminal prints: "Hello, rosmaster", you successfully setup the HARMONI full container.   

   *Note: We provide a bash script for launching multiple containers called run_compose.sh* 

 

3. Open a new terminal. To get into this container, you will need to run the following to mount a terminal into the container: 

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

   The `roscore` of QT robot is alreaedy set on the PI (which is the ros master).  

    Switch to qt branch

   ``` 
   git checkout robot/qt
   ``` 

### Configuration of Harmoni on the NUC 


Follow the instructions you find [here](../Configuration/Cloud-Services) for configuring the cloud services.  


 
## Run HARMONI on the PI 

 

1. **(optional)** In order to run with window forwarding on linux use: 

   ```bash 
   xhost +local: 
   ``` 

2. Use docker compose to launch the complete system (will build if necessary, use --build to force): 

   ```bash 
   docker-compose -f docker-compose-qt-pi.yml up 
   ```  

   If the terminal prints: "Hello, rosmaster", you successfully setup the HARMONI full container.   

   *Note: We provide a bash script for launching multiple containers called run_compose.sh* 

3. Open a new terminal. To get into this container, you will need to run the following to mount a terminal into the container: 

   ```bash 
   docker exec -it harmoni_hardware bash 
   ```  

   If this is successful, you will see: 

   ```bash 

   Hello! Welcome to Harmoni 

   11:47 AM [develop] ~/harmoni_catkin_ws/src/HARMONI : 
   root@harmoni_hardware#  
   ```  
    Switch to qt branch

   ``` 
   git checkout robot/qt
   ``` 


### Configuration of Harmoni on the PI 

For  setting up the Audio Hardware follow these steps. 

#### Speaker 

For testing the audio you have to start the speaker service on the NUC, and the player service on the PI. 

On the NUC (qtpc), run the command: 

``` 
roslaunch harmoni_speaker speaker_service.launch play:=false 
``` 

On the PI, check the `.asoundrc` file (located in `HARMONI/dockerfiles/config`) to ensure it refers to the appropriate sound device on your system. Determine which audio device you want to use by running `aplay -l` or `arecord -l` and update aforementioned `.asoundrc` file. *Note: make sure you can see hidden files*. If you are not using Docker, you will need to find or create the `.asoundrc` file in your home directory to configure sound in the same way. 

```bash 
cd dockerfiles/config 
nano .asoundrc 
# Update the hardware and soundcard id. 
```
The contents are shown below for working with the QT speaker: 

``` 
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

For testing the audio you cannot follow the testing command because the NUC is not connected to the speaker. For testing the speaker service please continue with the configuration on the PI. 

On the PI (qtrp), run the command: 

``` 
roslaunch harmoni_speaker speaker_play.launch 
``` 

#### Microphone 
Determine which capture audio device you want to use by running `arecord -l`. 

To choose a microphone you may need to use the command `arecord -l `  to see what devices are available.   

For example running `arecord -l`: 

``` 
card 1: PCH [HDA Intel PCH], device 0: ALC283 Analog [ALC283 Analog] 
``` 

In this case, to make your microphone working, change the [configuration file](https://github.com/interaction-lab/HARMONI/blob/develop/harmoni_sensors/harmoni_microphone/config/configuration.yaml) of your `harmoni_microphone` package. 

 

Generally, you need to modify only few parameters to get the microphone working. For example, check and substitute your _device_name_ and you microphone sample rate (_audio_rate_). 

The QT microphone configuration file has the following parameters: 

``` 
microphone: 
  default_param: 
    audio_format_width: 2 
    chunk_size: 1024 
    total_channels: 1 
    audio_rate: 16000 
    device_name: default 
    test_outdir: "$(find harmoni_microphone)/temp_data/test_example.wav" 
``` 

After updating your configuration file and restarting docker test the microphone with the following command on the PI: 

```bash 
rostest harmoni_microphone microphone.test 
``` 

If it works, you're good! Otherwise, re-do your [audio config](#audio), and try the above command until the audio works. 


#### Face Configuration 

To automatically start the face run the following. 

On the NUC (qtpc), run the command: 

``` 
roslaunch harmoni_face face_service.launch 
rosrun harmoni_face client_to_start_face.py 
``` 

On the PI (qtrp), run the command: 

 
``` 
rosrun harmoni_face server_to_start_face.py 
``` 
On the screen, the web page with the face will be displayed, and it continues refreshing until the face service has been started.

## Test HARMONI on QT 

On the NUC (qtpc), please run the following: 

``` 
run_qt_nuc.sh 
``` 
With this command, the core services (bot, tts, speaker, face) and the pattern test are started.

On the PI (qtrp), please run the following: 

``` 
run_qt_pi.sh 
``` 
With this command the speaker endpoint, and the face server (to automatically display it) are started.

The outcome is QT greeting you and displaying a face.  

