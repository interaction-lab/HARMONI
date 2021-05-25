# How to create an HARMONI package
Harmoni follows the standard ROS conventions for package structure.
Below you can find some info about how to create a package that follows the HARMONI Unit conventions.

## *CMakeLists.txt* and *setup.py* and *package.xml*
[MANDATORY]

## *launch* folder
[MANDATORY]

The *launch* folder stores the *.launch* file or files.
The structure of a *[your_package].launch* file is very similar to the *[your_package].test* file. The only difference is that the *[your_package].launch * file does NOT have the line that starts with <test ...>.

## *src* folder and *nodes* folder
[MANDATORY]

Here is the actual implementation of the service. Some have the *src* folder while some have the *nodes* folder

It is suggested to use the template provided (*harmoni_core/harmoni_common_lib/src/harmoni_common_lib/service.py.template*).

The structure is like this

In your package you may want to publish data so that other packages may use it or you may want to subscribe to some data published by other packages. For example you may want the audio stream coming from the *harmoni_microphone* package.

## README file
[HIGHLY SUGGESTED]

Here you should write what type of messages your package uses. If it uses standard messages this paragraph should be empty. 
There should be a brief description of what the package does.


## Tests
[HIGHLY SUGGESTED]

The *test* folder stores the tests created for the service.
There are three types of files in this folder: a *[your_package].test* file, a *rostest-[your_package].py* file and a *unittest-[your_package].py* file.

Let's start with the *.test* file.
This file is very similar to the *.launch* file that is stored in the *launch* folder.
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

The structure of the *rostest-[your_package].py* file is usually like:
```

```


## *config* folder and configuration.yaml file
[OPTIONAL]

The *configuration.yaml* file is a file where the user stores information useful to run the service correctly. For example, to run the harmoni_microphone service Harmoni must know what device is the correct one. So, the *configuration.yaml* file for the harmoni_microphone service stores the name of the device you want to use.

The *configuration.yaml* file stores the default_param, that is the default configuration to run the service. If you want you can create multiple parameters that have the same structure as the default_param.

The parameters in configuration.yaml are used in the _init_ method of in the service class.
This method is found in src/X_service.py or in....


## *msg* folder
[OPTIONAL]

This folder should be inside the package if the package doesn't use the standard messages. The description of the newly created message should be written in the readme.


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

### *constants.py* file
You must add the name of the service in the *constants.py* file that is in *harmoni_core/harmoni_common_lib/src/harmoni_common_lib/*.

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


