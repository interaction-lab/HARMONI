# Launching a Service 

## Basics

Launching services in HARMONI Follows the standard ROS convention:
```bash
roslaunch harmoni_$PACKAGE $PACKAGE_service.launch
```
For example:
```bash
roslaunch harmoni_tts tts_service.launch
```
Launching packages this way will allow you to verify if the package is correctly setup and installed. However to fully test a package, look at the "Running a test" Section. To use a package, you will need to interact with the service it exposes.

## Launching Services in Docker
See the Quickstart Docker section for details on getting started with docker. Once you have docker set up and are inside the container, you can run services as shown above.
Most services can be run in the 'full' container, however detectors often come with conflicting requirements. Therefore we choose to create seperate images for these modules, and run them in their own images.  
If you want to run a detector locally, use the same command but in the corresponding container. For example, for the stt package, use the ros_w2l container for running the script.  
Similarly, if there are seperate computers controlling the robot hardware, you may wish to have a seperate image for running on that computer. This is often named 'harmoni_hardware'.


## Launching Multiple Services at Once
Harmoni is supported by a long list of services which can be tedious to launch sequentially. To speed up the launching process, we provide a method for launching groups of services simultaneously with launcher.py and launcher.launch. This node creates custom launch files for the services which are specified in the harmoni_decision configuration.yaml. This file allows you to specify groups of services with a tag such as hardware or demo, which you can then launch as a group with the line: `roslaunch harmoni_decision launcher.launch service:='demo'`

The configuration is structured as:
```yaml
tag_name:
  namespace_enum: ["name"]
```
The namespace enum is defined in the harmoni_common_lib constants.py, see the existing configuration.yaml for examples.

### Roslaunch Aliases
We provide a set of alias shortcuts for launching your project quickly and easily. The full list is defined in dockerfiles/config/setup_script.sh.

```bash
rlhardwareservices
#alias rlhardwareservices="roslaunch harmoni_decision launcher.launch service:='hardware'"
```
