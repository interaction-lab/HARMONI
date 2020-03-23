# Run a full interaction

This has the robot track a user, ask their name (and verify it), and then closes. Makes use of face detection, speakers, microphone, and gestures.

[pi] roslaunch qt_robot_read_gesture read_gestures.launch
[pi] roslaunch qt_robot_speaker speaker.launch
[pi] roslaunch qt_robot_microphone microphone.launch
[nuc] roslaunch cordial_manager manager.launch