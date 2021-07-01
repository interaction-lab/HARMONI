# Launching Services

Launching services in HARMONI Follows the standard ROS convention:
```bash
roslaunch harmoni_$PACKAGE $PACKAGE_service.launch
```
For example:
```bash
roslaunch harmoni_tts tts_service.launch
```
Launching packages this way will allow you to verify if the package is correctly setup. However to fully test a package, look at the "Running a test" Section.



## Launching Services in Docker
Most services can be run in the 'full' container, however detectors often come with conflicting requirements. Therefore we choose to create seperate images for these modules, and run them in their own images.  
If you want to run a detector locally, use the same command but in the corresponding container. For example, for the stt package, use the ros_w2l container for running the script.  
Similarly, if there are seperate computers controlling the robot hardware, you may wish to have a seperate image for running on that computer. This is often named 'harmoni_hardware'.


### Roslaunch Aliases
We provide a set of alias shortcuts for launching your project quickly and easily. The full list is defined in dockerfiles/config/setup_script.sh.

```bash
rlhardwareservices
#alias rlhardwareservices="roslaunch harmoni_decision launcher.launch service:='hardware'"
```
After launching the ROS master, you can also launch the other detectors. For instance, go to the w2l terminal and launch speech-to-text detector with the following command:
```bash
rlspeech
#alias rlspeech="roslaunch harmoni_stt stt_service.launch"
```
or launch the face detector in the harmoni_visual_detector terminal
```bash
rlfacedetect
#alias rlfacedetect="roslaunch harmoni_face_detect face_detect_service.launch"
```