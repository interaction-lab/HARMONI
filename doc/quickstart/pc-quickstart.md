# Desktop Setup

Prerequisites: ROS Kinetic, Melodic, or Noetic Installed

_Warnings: 1) If you are installing on some OS other than Ubuntu, you may run into dependency issues that you will have to handle yourself. 2) HARMONI is written in Python 3, which can cause some hiccups if you are not used to running Python 3 in ROS Kinetic or Melodic_

_We recommend using docker whenever possible, however it is particularly challenging to set up a combination of python 3 and Melodic or Kinetic so we highly recommend [using docker](Docker-Quickstart) in this case. If you would like to set up a desktop install yourself, you may find [this guide](https://www.miguelalonsojr.com/blog/robotics/ros/python3/2019/08/20/ros-melodic-python-3-build.html) helpful._

1. Clone the repository:
   ```bash
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/interaction-lab/HARMONI.git
   cd ..
   ```

2. install dependencies:
   ```bash
   sudo apt-get install ros-$ROS_DISTRO-audio-common ros-$ROS_DISTRO-rosbridge-server
   pip install rospkg
   ```

3. Build harmoni core packages:
   ```bash
   catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 harmoni_common_lib harmoni_common_msgs harmoni_decision harmoni_pattern harmoni_recorder
   ```
   If you do not have catkin tools installed, follow [these instructions](https://catkin-tools.readthedocs.io/en/latest/installing.html) or also install from pip running:
   
   ```bash
   pip3 install --user git+https://github.com/catkin/catkin_tools.git
   ```

4. Build all packages: 
   
   After installing the requirements for each package you would like to use, you must build the remaining packages.
   ```bash
   catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
   source devel/setup.bash
   ```


## Additional Notes/Tips

For now this is a WIP. Some dependencies may need to be updated. We recommend installing with python3 from source, like [here](https://www.miguelalonsojr.com/blog/robotics/ros/python3/2019/08/20/ros-melodic-python-3-build.html)

We recommend extending the blacklist packages to include those shown in the following sample command:
```bash
catkin config --init -DCMAKE_BUILD_TYPE=Release --blacklist rqt_rviz rviz_plugin_tutorials librviz_tutorial turtlesim cv_bridge rqt_image_view turtle_actionlib turtle_tf2 turtle_tf rviz rviz_python_tutorial python_orocos_kdl --install
```

