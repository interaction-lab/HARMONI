# Testing

## Package Testing

For testing the packages you created or those provided by the HARMONI team, you can easily do it following the instructions below. You can both test the single package and then test if it is correctly integrated with the whole system

### Testing Design

There are three levels of testing:
- Unit Testing - _Do individual classes/functions run as expected?_
- Package Testing - _Do packages perform the function requested?_
- Integration Testing - _Do packages communicate with each other as expected?_

Package testing for sensors, actuators, and dialogues is ideally performed on real hardware. Some testing using simulation and/or rosbags in place of hardware is also acceptable.

Unit testing and integration testing are performed during building. Every package will have its own unit tests using pytest. Integration tests will be run during docker builds.

Package tests should force the package to perform some subset of the functionality the package provides and verify the output.

  - harmoni_actuators: _Should force the actuator to do something and verify the correct message is published._
    - harmoni_face
    - harmoni_speaker
    - harmoni_tts
    - harmoni_web
    - etc.
  - harmoni_detectors - _Should take a sample input and output the appropriate detection._
    - harmoni_face_detect
    - harmoni_stt
    - etc.
  - harmoni_dialogues - _Should take a sample phrase and return the expected response._
    - harmoni_bot
  - harmoni_sensors - _Should read from a sensor_
    - harmoni_camera
    - harmoni_microphone
    - etc.
  - harmoni_core - _Packages are tested during integration._


### How to test a single package

For testing a single package (a new one, or the HARMONI one) go to the terminal of your container (harmoni_core if you added a package in HARMONI repo or ros_hardware terminal if you want to test a hardware package) and run:

~~~
$ roslaunch harmoni_$PACKAGE harmoni_$PACKAGE_service.launch test:=true
~~~

For example go to the harmoni_core terminal and command:
~~~
$ roslaunch harmoni_tts tts_service.launch test:=true
~~~
or go to the ros_hardware terminal and run:
~~~
$ roslaunch harmoni_speaker speaker_service.launch test:=true
~~~
When you set the arg _test_ to True, the service uses the default _test_input_ value.
For each package, here the default value provided by the Harmoni team:

| Service  |Type  | Definition  |  Default value |
|---|---|---|---|
| harmoni_tts  | string   | Text to synthesize  | My name is HARMONI  | 
| harmoni_bot  |string   |Input text for dialogue system   | Hey  |  
| harmoni_face  |string   |Json format which contains information about the face expression (i.e., visemes and facial AUs)  | "[{'start': 0.075,'time': 2, 'type': 'viseme', 'id': 'POSTALVEOLAR'},{'start': 0.006, 'time': 2,  'type': 'action', 'id': 'happy_face'}]"  |  
| harmoni_web  |string   |Json format which contains information about div element to display on the WebApp| "{'component_id':'container_1', 'set_content': 'null'}"|  
| harmoni_speaker  |string   |File path to reproduce | "/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_tts/temp_data/tts.wav"|  
| harmoni_stt  | string   | File path to trascribe  |"$(find harmoni_stt)/temp_data/test.wav"  | 

For _harmoni_sensors_, no input value is provided because when you test them (e.g., harmoni_microphone), you only start the service.


### Test integration package with HARMONI

TODO