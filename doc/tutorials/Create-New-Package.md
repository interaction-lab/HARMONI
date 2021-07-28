# How to create an HARMONI package
Harmoni follows the standard ROS conventions for package structure.
Below you can find some info about how to create a package that follows the HARMONI Unit conventions.

[General info on how to create a ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).
Notice that HARMONI uses *harmoni_catkin_ws* as workspace instead of *catkin_ws* described in the tutorial.

## *CMakeLists.txt* and *setup.py* and *package.xml*
[MANDATORY]

These files store all the information needed to correctly build a package with its dependencies.
These files are very similar among packages, so it is suggested to copy them from another package and modify the package name/path.
To build a package use
> catkin build [your_package]

For more info see: [CMakeLists.txt documentation](http://wiki.ros.org/catkin/CMakeLists.txt)

## *launch* folder
[MANDATORY]

The *launch* folder stores the *.launch* file or files.
The structure of a *[your_package].launch* file is very similar to the *[your_package].test* file. The only difference is that the *[your_package].launch * file does NOT have the line that starts with <test ...>.

## *src* folder and *nodes* folder
[MANDATORY]

Here is the actual implementation of the service. Some packages store the code related to service class in a *src* folder while some have a *nodes* folder.
It is suggested to use the template provided (*harmoni_core/harmoni_common_lib/src/harmoni_common_lib/service.py.template*).

*[your_package]_service.py* contains the service implementation.

Some services act on a per request basis, meaning that they receive some optional data, they do something with it and they return a response.
For example, the TTS service may receive some text in input and its job is to produce an audio file from the written text.
Services that act on a per request basis must implement the *request* method.

The *request* method return this type of message : {"response": self.state, "message": self.result_msg} , where self.state describes the state of the service (see *harmoni_core/harmoni_common_lib/src/harmoni_common_lib/constants.py* ) while self.result_msg stores the output result.

There are other services that, once started, keep on running.
For example, the microphone service, once started, keeps sending audio data.
These kind of services implement the *start* and *stop* methods.

## Data
The general policy is that you use the HARMONI service system to send small commands and info messages among services.

However, if you have high density data (e.g. images and audio), do [publishing and subscription as normal ROS nodes](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
In this case, be sure to follow the HARMONI namespace guidelines so that multiple packages can publish/subscribe to known interfaces (see [Namespaces and the *constants.py* file section](#constants)). This approach is especially useful when there are multiple packages that provide the same data, as is the case with different types of STT services.


## README file
[STRONGLY SUGGESTED]
There should be a brief description of what the package does.
There should be an entry for each parameter set in configuration.yaml with a brief description and value. 
You should write what type of messages your package uses, if your package uses non-standard messages. 

```
| Parameters           | Definition | Values |
|----------------------|------------|--------|
|parameter_1           |            |        |
|parameter_2           |            |        |
|parameter_3           |            |        |
```


## Tests
[STRONGLY SUGGESTED]

The *test* folder stores the tests created for the service.
There are three types of files in this folder: a *[your_package].test* file, a *rostest-[your_package].py* file and a *unittest-[your_package].py* file.

The *.test* file is very similar to the *.launch* file that is stored in the *launch* folder.
*[your_package].test* is a file that connects the configuration parameters with the actual implementation of the service, which is stored in the *src* or *nodes* folders.
The file *[your_package].test* contains also the link to the actual test file, which is usually called *rostest_[your_package].py*.
The file *[your_package].test* tries to complete the task written in the *rostest_[your_package].py* file, which can succed or fail.

The structure of the *[your_package].test* file is usually like:

```
<launch>
    <rosparam file="$(find harmoni_[your_package])/config/configuration.yaml" subst_value="True"/>
    <param name="instance_id" value="default"/>
    
    <node pkg="harmoni_[your_package]" type="[your_package]_service.py" name="harmoni_[your_package]" output="screen"/>

    <test test-name="test_[your_package]" pkg="harmoni_[your_package]" type="rostest_[your_package].py" />
</launch>
```
Parameters can be specified in the *configuration.yaml* file or directly in the *.test* file. For example you could add a line to have an input parameter with the value "Hello".
```
    <param name="test_[your_package]_input" value="Hello"/>
```

Test files and launch files make use of namespaces when including config files. For this reason, it is important to follow the HARMONI namespace guidelines (see [Namespaces and the *constants.py* file section](#constants)).
If these are not added, it is possible for concurrently running services to overwrite eachother's params (e.g. in *harmoni_detectors/harmoni_face_detect/launch/face_detect_service.launch*).

<!-- The structure of the *rostest-[your_package].py* file is usually like:
```

``` -->


## *config* folder and configuration.yaml file
[OPTIONAL]

The *configuration.yaml* file is a file where the user stores information useful to run the service correctly. For example, to run the harmoni_microphone service Harmoni must know what device is the correct one. So, the *configuration.yaml* file for the harmoni_microphone service stores the name of the device you want to use.

The *configuration.yaml* file stores the default_param, that is the default configuration to run the service. If you want you can create multiple parameters that have the same structure as the default_param.

The parameters' values are then retrieved in service class implementation.


## *msg* folder
[OPTIONAL]

This folder should be inside the package if the package doesn't use the standard messages. The description of the newly created message should be written in the README.


## Other folders (e.g. the *web* folder in harmoni_web or the *temp_data* folder in harmoni_camera)
[OPTIONAL]

If needed, you may create additional folders.


## HARMONI conventions to follow

### Where should you place your package?
It depends on what does your package do.

Your package should be put in the folder corresponding to the type of service you have created (e.g. *actuators* "do" something, *sensors* retrieve data...).

- Actuators -> *harmoni_actutators* 

- Detectors -> *harmoni_detectors* 

- Sensors -> *harmoni_sensors* 

- Dialogues -> *harmoni_dialogues* 

### <a name="constants"></a>Namespaces and the *constants.py* file  
The HARMONI namespace guidelines require you to add the name of the service in the *constants.py* file that is in *harmoni_core/harmoni_common_lib/src/harmoni_common_lib/*.

A new line should be put in the Enum corresponding to the type of service you have decided for your package.

If your service is an actuator a line with the name of your service should be added in the ActuatorNameSpace(Enum), if it is a detector in the DetectorNameSpace(Enum) and so on...

For example, if your service is of type *sensor*:

```
class SensorNameSpace(Enum):
    microphone = "/harmoni/sensing/microphone/"
    camera = "/harmoni/sensing/camera/"
    [your_service] = "/harmoni/sensing/[your_service]"
```

## Troubleshooting
- Check that your package has built or do:
> catkin build [your_package]
- Check that the *.py* files (in *src* or *nodes*) in your package are executable
