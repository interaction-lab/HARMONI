# Setting Up A Robot

STUB -- need to define interfaces that would be attached to


# MISTY robot (API based) V0.1

In order to be able to run HARMONI on an API based robotic platform, a few modifications are needed. 

First of all, the IP of the robot, which is needed in most of the _service file, is saved as a ros parameter in the initialization of the harmoni_core/harmoni_common_lib/service_manager.py  class; and then retrieved directly in the implementation of the actual service class.

Here are listed the packages and the files that have been modified and added in order to make HARMONI work on the robot.

The docker-compose-full.yml file have been modified in order to be able to run docker in distributed mode, and enable communications between harmoni inside the docker and external networks. In particular, port forwarding paths are necessary, and the network ros_net must not be specified. The network mode must be set to "host". 
## Usage

## Parameters

## Testing

## References


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
TODO


In this package, the speaker_service_misty.py has been added, and the launch and test files have been modified consequently.

In it, the DO operation that was used before for ordering the robot to play a recording, has been replaced by a REQUEST operation, which is more in line with the API nature of the interaction. 

It is important to remember that when a REQUEST operation is called, it is necessary to set the flag response_received = True when the API call is over. 

Since Misty needs to recieve the files in base64 format, the file_path_to_audio_data function has been modified to convert the wav file in that format. Currently, the implementation of the robot call providing directly the data is not implemented, and the data must always be given to the node as a wav file. In case free data are provided, the node assumes that it is the tts package calling the speaker, and so retrieve the data where the tts node saves the file as a temporary .wav file.

The function prepares the url and the payload for the API call has been added to the file, and then the request is forwared to the robot. If the call for any reason timed-out, a second call is made with a longer timeout (sometimes connection problem can happen), and if also that one fails, the function return setting the done flag to false.

It is important to note that, since the wav file can be heavier than the maximum file size for the parameters, the payload must be passed as data and not as a parameter to the API call.

An argument robot_ip is provided in the service_manager class. 
In misty, the file must either be encoded in base64 or provided as file. In the speaker_service_misty class the data are converted after being uploaded from a .wav file, either that they are provided to the class directly or via path. 

To reproduce the file the flag ImmediatelyApply has been set to True in the API call.
The flag OverwriteExisting makes the new file replace the old, since the file is always called with the same name. 
Another possible way of doing this was to separate into two different API call, SaveAudio with ImmediatelyApply set to false and then PlayAudioClip.

When replacing harmoni's speaker_service with speaker_service_misty, the type of action that the class performs has been changed from DO to REQUEST: this was done because conceptually we are requesting to an external service (the robot) to perform an action, and not performing the action directly inside HARMONI. This change requires to modify everything that interfaces with the speaker service 

Be sure to add the correct robot_ip argument to the launch file if you want to test the case with no exception, or to add a mock one if you to test the exception.

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


### HARMONI face: Using Misty API
An argument robot_ip is provided in the service_manager class. 

A class misty_caller has been added to implement automatically a request to display the webView of harmoni face on robot's screen.
