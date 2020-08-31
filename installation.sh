# Install components needed for HARMONI

# We recommend using a clean install of ROS with python 3





# If you did not complete the installation of ROS build tools uncomment the following:
# sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
# sudo apt install python-rosdep
# sudo rosdep init
# rosdep update

# Install the prerequisites
sudo apt-get install -y --no-install-recommends nodejs 
sudo apt-get install -y --no-install-recommends npm
sudo npm install http-server -g
ln -s /usr/bin/nodejs /usr/bin/node
sudo apt-get install -y --no-install-recommends python-catkin-tools \
    ros-$ROS_DISTRO-rosbridge-server \
    portaudio19-dev \
    vorbis-tools \
    python3-scipy \
    python3-numpy \
    python3-empy \
    python3-soundfile \
    ffmpeg \
    python3-opencv \
    python3-pyaudio


# Other prerequisites?
sudo apt-get install -y --no-install-recommends libboost-all-dev \
    libgstreamer1.0-0 gstreamer1.0-dev gstreamer1.0-tools gstreamer1.0-doc \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev \
    # ros-$ROS_DISTRO-dynamixel-msgs \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-audio-common \
    ros-$ROS_DISTRO-audio-common-msgs \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-opencv \
    build-essential \
    python-rospkg

python3.6 -m pip install -r ./requirements.txt

# Clone the additional repositories
cd ../..
git -C src clone -b develop https://github.com/interaction-lab/HARMONI-PC.git 
git -C src clone -b master https://github.com/ros-drivers/audio_common.git
# git -C src clone -b $ROS_DISTRO https://github.com/ros-perception/vision_opencv.git
git -C src clone -b $ROS_DISTRO-devel https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git

# Build all packages
catkin init 
catkin config --extend /opt/ros/$ROS_DISTRO \
    -DPYTHON_EXECUTABLE=/usr/bin/python3.6 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build 