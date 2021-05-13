Testing
================================

For testing the packages you created or those provided by the HARMONI team, you can easily do it following the instructions below. You can both test the single package and then test if it is correctly integrated with the whole system

| There are three levels of testing:
- Unit Testing - *Do individual classes/functions run as expected?*
- Package Testing - *Do packages perform the function requested?*
- Integration Testing - *Do packages communicate with each other as expected?*

Package testing for sensors, actuators, and dialogues is ideally performed on real hardware. Some testing using simulation and/or rosbags in place of hardware is also acceptable.

Unit testing and integration testing are performed during building. Every package will have its own unit tests using pytest. Integration tests will be run during docker builds.

Package tests should force the package to perform some subset of the functionality the package provides and verify the output.

- harmoni_actuators: *Force the actuator to do something and verify the correct message is published.*
    - harmoni_face
    - harmoni_speaker
    - harmoni_tts
    - harmoni_web
    - etc.
- harmoni_detectors - *Take a sample input and output the appropriate detection.*
    - harmoni_face_detect
    - harmoni_stt
    - etc.
- harmoni_dialogues - *Take a sample phrase and return the expected response.*
    - harmoni_bot
- harmoni_sensors - *Read from a sensor*
    - harmoni_camera
    - harmoni_microphone
    - etc.
- harmoni_core - *Tested during integration.*
