# Full Setup


Prerequisites: ROS Kinetic, Melodic, or Noetic Installed

1. Clone the repository:
   ```bash
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/interaction-lab/HARMONI.git
   cd ..
   ```

2. install dependencies:
   ```bash
   sudo apt-get install ros-$ROS_DISTRO-audio-common
   ```

3. Build harmoni core packages:
   ```bash
   catkin build harmoni_common_lib harmoni_common_msgs harmoni_decision harmoni_pattern harmoni_recorder
   ```
   If you do not have catkin tools installed, follow this instruction (https://catkin-tools.readthedocs.io/en/latest/installing.html) or also install from pip running:
   
   ```bash
   pip3 install --user git+https://github.com/catkin/catkin_tools.git
   ```
#### TODO
For now this is a WIP. We recommend installing with python3 from source, like [here](https://www.miguelalonsojr.com/blog/robotics/ros/python3/2019/08/20/ros-melodic-python-3-build.html)

We recommend extending the blacklist packages to include those shown in the following sample command:
```bash
catkin config --init -DCMAKE_BUILD_TYPE=Release --blacklist rqt_rviz rviz_plugin_tutorials librviz_tutorial turtlesim cv_bridge rqt_image_view turtle_actionlib turtle_tf2 turtle_tf rviz rviz_python_tutorial python_orocos_kdl --install
```

We provide the script installation.sh to set up after ROS is installed.

