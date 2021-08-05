# HARMONI Gesture

The HARMONI gesture module is responsible for the actuation of gestures.
This package can be test either on the robotic platform ($ROBOT), or using a RVIZ simulator. The RVIZ simulator is now only working for kinetic distro and not noetic. 
The HARMONI gesture packages includes: 
- the HARMONI wrapper (gesture_service.py)
- the robot interface ($ROBOT_gesture_interface.py) which depends on the robot
- the simulator for publishing joints ($ROBOT_joint_state_publisher.py) which depends on the robot

To date, the HARMONI gesture supports:
- QT Robot
- Misty Robot (working only with the robot and no simulator is available because no model has been provided by the company)

Feel free to contribute to this package to add interfaces for new robots.

The name of the available gesture should be collected in the folder _data_ of the harmoni_gesture package. 
For QT for example, the available gestures are:
- QT/angry
- QT/bored
- QT/breathing_exercise
- ...
and all the available xml files in the data/QT folder.

## Usage

The following documentation refers to the gesture request.

The API for Gesture has:

- Request Name: ActionType: DO
- Body: data(str)
- Response:
    - response (int): SUCCESS, or FAILURE
    - message (str): empty string because no response is provided for a DO action

The body string is a list of object that could be either:
1. object containing the results of the TTS synthetization with the following items:

    |Key| Definition| Value Type |
    |---|-----------|------------|
    | behavior_data  | Data which includes information about facial expressions, action units, and gestures          |  str     |

    If you request to the TTS polly a sentence which included also the gesture (written between start: *$gesture_name*), the gesture service is able to parse it. It also synchronize the gesture according to the words timing.
    For example, if you want to synthesize the sentence: "Hello my name is QT" adding a waving gesture, in the case of QT you can use the following: "Hello *QT/hi* my name is QT". The gesture "hi" will be performed just after the word "Hello", because it reflects the order in the sentence.

2. object conaining the direct command for the gesture:

    |Key| Definition| Value Type |
    |---|-----------|------------|
    | name  | name of the gesture (e.g., "QT/bye") |  str; $gesture_name     |
    | timing  | duration of a gesture |  int[sec]  |


## Parameters

Parameters input for the gesture service corresponds the the $ROBOT_param which includes:

|Parameters| Definition| Value |
|---|-----------|------------|
| path  | path which containes the gestures  |  str; in the QT case: "$(find harmoni_gesture)/data"    |
| rate  | speed of gesture  |  int; 10   |
| robot_joint_topic  | topic which publishes the joints position in degree  |  str; in the QT case: "qt_robot/joints/state"    |
| robot_joint_topic_radiands  | topic which publishes the joints position in radians  |  str; in the QT case: "qt_robot/joints/state_rad"    |str; in the QT case: "$(find harmoni_gesture)/data"    |
| robot_gesture_topic  | topic of the robot which performs the real gesture on the robot  |  str; in the QT case: "qt_robot/gesture/play"   |
| timer_interval  | time interval for gesture (in seconds) |  int; 0.01  |

## Testing

To test that the gesture has been configured properly, use the command ```rostest harmoni_gesture gesture.test``` which will play a short phrase through the configured device. This test checks that the joints can be properly controlled to run through gestures.

TODO:
- Testing to perform on the real robot

## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_gesture.html)