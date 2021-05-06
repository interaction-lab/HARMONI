# Architecture
*************

Todo Check Out

## HARMONI Repository
The HARMONI repository is organized into the following packages, based on the fundamental capabilities needed for human-robot interaction:
  - harmoni_actuators - _controlling hardware (e.g. motors, screens, speakers)_
  - harmoni_core
    - harmoni_common_lib - _defines the Harmoni unit and helper functions_
    - harmoni_common_msgs - _defines harmoni messages and actions_
    - harmoni_decision - _highest level controller, can play patterns or individual units_
    - harmoni_pattern - _middle level decision player, defines sequences or patterns for dialog_
    - harmoni_recorder - _records interactions (WIP)_
  - harmoni_detectors - _extracting useful information from sensor signals (e.g. transcriptions, facial locations, etc.)_
  - harmoni_dialogues - _processing user speech (text) and return robot speech (text)_
  - harmoni_sensors - _reading physical sensors and publishing sensor streams_

[[/images/PackageOrganization.png]]

## Harmoni Packages
Harmoni follows the standard ROS conventions for package structure, as follows:
``` bash
$NAME
    |__ config
       |__ configuration.yaml
    |__ launch
       |__ $NAME_service.launch
    |__ nodes
        |__ $NAME
            |__ $NAME_service.py
            |__ *.py
    |__ src
        |__ $NAME
            |__ *.py
    |__ tests
        |__ *.py
    |__ CMakeLists.txt
    |__ package.xml
    |__ setup.py
```
For setting the parameters of the different packages, the HARMONI team provided a configuration.yaml file, which can be changed according to the user setup requirements.

## HARMONI Unit

[[/images/HarmoniUnitUML.png]]

Nearly everything in HARMONI is a ROS node called a HARMONI unit. The HARMONI unit consists of two classes, a Service Server and a Service Manager, which standardize the interface for a given node as shown above.

The Harmoni Unit standardizes the communication and control interface for each node, allowing other nodes to control a unit with minimal knowledge of the internal workings. 
