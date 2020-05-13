# HARMONI
Controller code for Human And Robot Modular OpeN Interactions

## Setup Instructions without docker

1. Install Ubuntu 16.04 on your computer

    http://releases.ubuntu.com/16.04/

2. Install ROS kinetic (full version)

    http://wiki.ros.org/kinetic/Installation/Ubuntu

3. Install Prerequisites
    ~~~~
    $ sudo apt-get install nodejs
    $ sudo apt-get install npm
    $ sudo npm install http-server -g
    $ ln -s /usr/bin/nodejs /usr/bin/node
    $ sudo apt-get install python-catkin-tools ros-kinetic-rosbridge-server portaudio19-dev vorbis-tools python3-scipy python3-numpy python3-empy python3-soundfile ffmpeg python3-opencv python3-pyaudio 
    ~~~~

4. Set up ROS with Python3:
    - Basic setup: execute the scripts with the shebang:
       ~~~~
       #!/usr/bin/env python3
       ~~~~
        If you get an error about some missing yaml library run in a terminal the following commands:
        ~~~~
        $ sudo apt-get install python3-pip python3-yaml
        $ sudo pip3 install rospkg catkin_pkg
        ~~~~
    - For additional setup, follow the instructions reported here:
        https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674


5. Set up AWS account following these steps (or request access to the lab account from a PhD): 

    Create an Amazon Web Services account. AWS has a 1 year free trial that includes a limited number of Polly usages.
    Keep this in mind so you do not get charged money at the end of the year.

    Once you've created an account, create an IAM user to access Polly.

      * Give the IAM user access permissions to AWS Polly.
      * Give the IAM user access keys. Be sure to save the secret key as you only have one chance to look at it.
      
    https://docs.aws.amazon.com/cli/latest/userguide/install-linux.html - Use this link to install the AWS CLI on your PC.
    
    Then, in the terminal,
    ~~~~
    $ sudo apt-get install awscli
    $ aws --version
    $ aws configure
    # Enter the IAM user access and secret keys here.
    ~~~~

6. Clone the repository in <your_catkin_workspace>/src

    ~~~~
    $ git clone https://github.com/interaction-lab/HARMONI.git 
    $ git clone https://github.com/interaction-lab/HARMONI-PC.git (if you want to use the pc packages)
    ~~~~

7. Install packages:
    ~~~~
    $ sudo apt-get install ros-kinetic-audio-common-msgs
    ~~~~

8. Make everything:

    ~~~~
    $ cd ..
    $ catkin init 
    $ catkin config -DPYTHON_EXECUTABLE:=/usr/bin/python3
    $ catkin build 
    ~~~~

## Setup Instructions with docker
1. In order to run with window forwarding on linux use:
```
    xhost +local:
```

2. Use docker compose to launch the complete system (will build if necessary, use --build to force) for developing:
```
    docker-compose -f docker-compose-harmoni-dev.yml up
```


## Test instructions (with HARMONI_PC)
Before testing these packages, you should follow the HARMONI_PC setup instructions.
Follow these steps for testing each service of HARMONI.
1. Choose which service ("service_to_test") you want to test among the following:
    - microphone (HARMONI-PC)
    - speaker (HARMONI-PC)
    - face (HARMONI-PC)
    - camera (HARMONI-PC)
    - tts
    - lex
    - web
    - stt
2. Open 4 terminals:
    ~~~~
    $ roscore
    ~~~~
    ~~~~
    $ roslaunch harmoni_decision routers.launch
    ~~~~
    ~~~~
    $ roslaunch harmoni_decision services.launch
    ~~~~
    (all the services listed above are launched)
    ~~~~
    $ roslaunch harmoni_decision behavior_interface.launch test_service:= "service_to_test"
    ~~~~
3. Open the webpages for both face and web. You find the url in the terminal
4. Refresh the page for successfully setting up the face and web servers (it will be automatically handle autostart file)
5. Verify if the goal has been successfully received

### Test custom messages
The above instructions enables to run some default messages of each service. If you want to test some customized messages, check this documentation for the message format:

| Service              | Argument name | Message format | Default |
|----------------------|---------------|----------------|---------|
|camera            |None          |None        | None|
|microphone            |None          |None        | None|
|speaker                 | wav_file           | string: path of file to save     | "/home/username/catkin_ws/src/HARMONI/harmoni_tts/temp_data/tts.wav"/|
|face              |   face_input         | string: [{start: int, time: int, type: string (i.e.,action, viseme, or word), id: string}]     | "[{'start': 0.075, 'time': 2,'type': 'action', 'id': 'QT/point_front'}, {'start': 0.075,'time': 2, 'type': 'viseme', 'id': 'POSTALVEOLAR'},{'start': 0.006, 'time': 2,  'type': 'action', 'id': 'happy_face'}]"|
|tts                |   tts_input_text         |     string   |"My name is HARMONI"|
|stt      |  TODO          |     TODO   |TODO|
|lex      |  dialogue_input_text          |     string   |"Hey"|
|web      |  display_input          |     string:{component_id : string, set_content: string}   |"{'component_id' : 'container_1', 'set_content': ' ' }"|

Here an example with the instruction for testing custom message:

1. Open 4 terminals:
    ~~~~
    $ roscore
    ~~~~
    ~~~~
    $ roslaunch harmoni_decision routers.launch
    ~~~~
    ~~~~
    $ roslaunch harmoni_decision services.launch
    ~~~~
    (all the services listed above are launched)
    ~~~~
    $ roslaunch harmoni_decision behavior_interface.launch test_service:= "tts" tts_input_text:="Hi, nice to meet you"
    ~~~~
2. Open the webpages:
    127.0.0.1:8080/index.html (face port) 
    127.0.0.1:8081/index.html (web port)
3. Refresh the page for successfully setting up the face and web servers (it will be automatically handle autostart file)


