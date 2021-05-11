# Package Level Organization

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
