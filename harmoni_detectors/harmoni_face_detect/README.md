# harmoni_face_detect

The face detection module works by subscribing to an image topic and detecting faces in the recieved images, which are then published to another topic.

## Usage 

Requires python2.6 - python3.6. Python 3.7 and newer will require dlib compilation from source.

Also note that there may be issues if you're running with Python3 but the python2 paths are overshadowing your opencv install (e.g. $PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages). One workaround is simply to prepend the python2.7 path with your own opencv path (e.g. export PYTHONPATH=/usr/bin/lib/opencv3_path:PYTHONPATH . The opencv instructions on this page may also work: https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674

In the future, an easier workaround might be to just install under ROS Noetic or ROS2 which are both based on python3 out of the box.

## Parameters
## Testing
## References
[Documentation](https://harmoni.readthedocs.io/en/latest/packages/harmoni_face_detect.html)

Based on work from https://github.com/fyr91/face_detection

