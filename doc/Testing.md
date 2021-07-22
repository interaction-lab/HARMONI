# Testing



For testing the packages you created or those provided by the HARMONI team, you can easily do it following the instructions below. You can both test the single package and then test if it is correctly integrated with the whole system

There are three levels of testing:
- Unit Testing - _Do individual classes/functions run as expected?_
- Package Testing - _Do packages perform the function requested?_
- Integration Testing - _Do packages communicate with each other as expected?_

## Unit Testing

Unit testing and integration testing are performed during building. Every package will have its own unit tests using pytest. Integration tests will be run during docker builds.

## Package Testing

Package testing for sensors, actuators, and dialogues is ideally performed on real hardware. Some testing using simulation and/or rosbags in place of hardware is also acceptable. Package tests should force the package to perform some subset of the functionality the package provides and verify the output.

  - harmoni_actuators: _Force the actuator to do something and verify the correct message is published._
    - harmoni_face
    - harmoni_speaker
    - harmoni_tts
    - harmoni_web
    - etc.
  - harmoni_detectors - _Take a sample input and output the appropriate detection._
    - harmoni_face_detect
    - harmoni_stt
    - etc.
  - harmoni_dialogues - _Take a sample phrase and return the expected response._
    - harmoni_bot
  - harmoni_sensors - _Tead from a sensor_
    - harmoni_camera
    - harmoni_microphone
    - etc.
  - harmoni_core - _Tested during integration._

## Integration Testing

Integration tests may be largely similar to package tests, except that they will involve more than one package and in certain instances may require user interaction or mock user interaction.