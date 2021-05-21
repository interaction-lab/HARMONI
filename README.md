![logo](logo.svg)

Welcome to HARMONI (Human And Robot Modular OpeN Interactions)! HARMONI is a ROS based tool for creating and controlling human-robot interaction. HARMONI is an open source (MIT License) tool meant to speed up development, collaboration, and experimentation for the HRI community. 

Repository: https://github.com/interaction-lab/HARMONI

Wiki: https://github.com/interaction-lab/HARMONI/wiki


### Setup Instructions

For setup and installation please jumpt to our getting started page [here](https://github.com/interaction-lab/HARMONI/wiki/Getting-started).



# Overview

Human And Robot Modular OpeN Interactions (HARMONI) is a comprehensive tool containing all the components and capabilities you need to quickly get your social interaction up and running on a robot. 

HARMONI packages the additional capabilities needed for social human-robot interaction neatly on top of ROS by wrapping and integrating several state of the art libraries in the domains of natural language understanding, dialog, object and face detection, and decision trees. HARMONI is the glue that integrates these disparate packages on top of ROS in a standardized way so you don't have to. Just [install our docker images](https://github.com/interaction-lab/HARMONI/wiki/Setup) and [configure the capabilities you need](https://github.com/interaction-lab/HARMONI/wiki/Usage) for your interaction.

HARMONI is built to be Robot-Agnostic, Modular, and Composable. By building on top of ROS and wrapping the whole platform in Docker, HARMONI should work on most hardware platforms and operating systems. By keeping to well defined interfaces with loose coupling and encapsulation HARMONI is a modular distributed system. This modularity further enables the system to be composed to suit the user's needs just through configuration files.

# HARMONI Integrations

Most robots need some combination of the abilities to sense, plan, and act. For complex social interactions these abilities can be further disected to the following categories: sense, detect, dialogue, plan, and act. Although it is not the only way to break down a social robot architecture, it is helpful to group the modules in this way. Listed modules have already been integrated with HARMONI and can be used seperately or in place of one another.

## Sense 
Sensing involves reading sensor data, such as audio, video, and depth sensors. Further sensor integrations will be added as needed or can be contributed by the community.

- PyAudio

- OpenCV

## Detect
Detecting involves taking in sensor data and detecting some signal from it. This includes transcribing speech, detecting objects, and detecting faces. Other detectors such as turn-taking, affect, and engagement will be added eventually.

- Wave2Letter

- DLIB

- Google StT

## Dialogue
Dialogue can be integrated with sensing and detecting, but can also stand on its own. Dialogue must be able to take in speech or text and return the appropriate speech or text.

- Amazon Lex

- Google Dialogflow

- Rasa

## Core (Plan and Control)
The core is tightly coupled with all of the other modules as it controls the other modules according to the state. The planner keeps track of the state and directs the other modules to start, stop, or do whatever is needed. HARMONI has its own core modules for planning, divided into a high level decision maker and a lower lavel pattern executor, however it also integrates with the following:

- PyTrees

- Unity


## Act
Actuation includes control of the motors, displays, and displays. 

- Amazon Polly

- Cordial

- PyAudio

## Robots
The following is a growing list of robots that HARMONI has be used on:

- QT


# Call for Contributions
We hope you will enjoy and find use in our project. We always welcome developments on new robots or with modules. To learn more about developing new modules please see [here](https://github.com/interaction-lab/HARMONI/wiki). We also have a backlog of [issues](https://github.com/interaction-lab/HARMONI/issues) and welcome pull requests.


## TODO
- [ ] Add links to integrated modules
- [ ] Test HARMONI on:
    - [ ] Quori
    - [ ] Kiwi
    - [ ] Curie
    - [ ] Blossom

## Using HARMONI on MISTY robot (API based) V0.1

In order to be able to run HARMONI on an API based robotic platform, a few modifications are needed. 

First of all, the IP of the robot, which is needed in most of the _service file, is saved as a ros parameter in the initialization of the harmoni_core/harmoni_common_lib/service_manager.py  class; and then retrieved directly in the implementation of the actual service class.

Here are listed the packages and the files that have been modified and added in order to make HARMONI work on the robot.

The docker-compose-full.yml file have been modified in order to be able to run docker in distributed mode, and enable communications between harmoni inside the docker and external networks. In particular, port forwarding paths are necessary, and the network ros_net must not be specified. The network mode must be set to "host". 

### harmoni_actuators

#### harmoni_face

the service of this package does not need to be modified.
A "caller" node file has been added, it's only role is to call the "Display web page" API of the robot. 
The node needs to be added to the test and launch files.
The face has been adapted to better fit the robot display, modifying the web/src/js/face.js file.

#### harmoni_gesture

This part of the implementation is still not completed, so the documentation will be adjusted.
Two files have been added to the package: gesture_service_misty.py and misty_gesture_interface.py.
The first one is meant to replace the normal gesture_service file, and thus has been substituted in the launch and test files.
In it, the DO operation that was used before for ordering the robot to make a gesture, has been replaced by a REQUEST operation, which is more in line with the API nature of the interaction. 
It is important to remember that when a REQUEST operation is called, it is necessary to set the flag response_received = True when the API call is over. 
The parse_gesture_misty() function has been added to the file, and there the gesture described in the xml files in the data folder are processed and for each command composing the gesture an API call is made. A function which prepares the url and the payload for each possible API call has been added to the file, and then the request is forwared to the robot. If the call for any reason timed-out, a second call is made with a longer timeout (sometimes connection problem can happen), and if also that one fails, the function return setting the done flag to false.
Currently the misty_gesture_interface file only publishes some required parameters to ros, but we are planning on moving the api calls described above in this file, in the attempt to better resamble the previous work done on qt. The documentation will be updated consequently.
The xml files contained in data/Misty folder resemble the format for the API calls of the robot, adding only a time parameter which is used for choosing after how much time each single movemente should be performed. Be aware that the time for sending the call is not calculated in a strict manner, so the execution time may consequently be not completely precise.

#### harmoni_speaker

In this package, the speaker_service_misty.py has been added, and the launch and test files have been modified consequently.
In it, the DO operation that was used before for ordering the robot to play a recording, has been replaced by a REQUEST operation, which is more in line with the API nature of the interaction. 
It is important to remember that when a REQUEST operation is called, it is necessary to set the flag response_received = True when the API call is over. 
Since Misty needs to recieve the files in base64 format, the file_path_to_audio_data function has been modified to convert the wav file in that format. Currently, the implementation of the robot call providing directly the data is not implemented, and the data must always be given to the node as a wav file. In case free data are provided, the node assumes that it is the tts package calling the speaker, and so retrieve the data where the tts node saves the file as a temporary .wav file.
The function prepares the url and the payload for the API call has been added to the file, and then the request is forwared to the robot. If the call for any reason timed-out, a second call is made with a longer timeout (sometimes connection problem can happen), and if also that one fails, the function return setting the done flag to false.
It is important to note that, since the wav file can be heavier than the maximum file size for the parameters, the payload must be passed as data and not as a parameter to the API call.

#### harmoni_tts

this package has not been relevantly modified.

#### harmoni_web

this package has not been relevantly modified.

### harmoni_core

#### harmoni_common_lib

the only relevant modification is the addition of the IP of the robot, saved as a ros parameter in the initialization of the harmoni_core/harmoni_common_lib/service_manager.py class. In future this parameter is expected to be passed throught a configuration file. (TODO)

#### harmoni_decision

the configuration file has been modified, adding the parameter   gesture: ["misty"] that was previusly commented.

#### harmoni_pattern

two interactions have been added: pattern_scripting/misty_demo.json pattern_scripting/misty_interaction.json  and (one for tests and one for the actual interactions). The interactions name must be added also to configuration file too. currently the demos are sort of "mock-ups" of real interactions. 
The node has not been relevantly modified.

### harmoni_detectors

this package has not been modified yet.

### harmoni_dialogues

this package has not been modified yet.

### harmoni_sensors

this package has not been modified yet.