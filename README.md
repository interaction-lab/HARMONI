# HARMONI
Controller code for Human And Robot Modular OpeN Interactions

## Setup Instructions

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
$ sudo apt-get install ros-kinetic-rosbridge-server vorbis-tools python3-scipy python3-numpy python3-empy
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

## Test instructions (with HARMONI_PC)
Before testing these packages, you should follow the HARMONI_PC setup instructions.
### Test wrapper
Open the terminal and launch the test:
~~~~
$ roslaunch harmoni_decision test.launch test_service:="service_to_test"
~~~~

### Test separately wrappers (suggested)
Open 4 terminals:
1. Terminal 1:
~~~~
$ roscore
~~~~
2. Terminal 2:
~~~~
$ roslaunch harmoni_decision router.launch
~~~~
3. Terminal 3
~~~~
$ roslaunch harmoni_decision services.launch
~~~~
4. Terminal 4
~~~~
$ roslaunch harmoni_decision behavior_interface.launch test_service:= "service_to_test"
~~~~

The "service_to_test" for HARMONI and HARMONI_PC are the following:
- microphone
- speaker
- face
- tts
- lex