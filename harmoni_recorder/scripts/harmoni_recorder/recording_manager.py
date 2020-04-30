#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from audio_common_msgs.msg import AudioData

class HarmoniRecordingManager():
    """
    The recording manager aims at storing the information collected by the sensors
    """

    def __init__(self, manager_name,child_names):
        """ Init recording manager"""
        rospy.loginfo("Init recording the data from the sensors")
        self.audio_child = child_names["audio_data"]
        """ Init subscribers"""
        for child in self.audio_child:
            rospy.Subscriber("/harmoni/sensing/listening/"+child, AudioData, self._audio_data_callback)

    def _audio_data_callback(self, data):
        """ Do something when audio data has been received """
        rospy.loginfo("The audio data received is %s" %data)
        return

    def _video_data_callback(self, data):
        """ Do something when video data has been received """
        rospy.logdebug("The video data received is %s" %data)
        return


def main():
    try: 
        manager_name = "recording"
        rospy.init_node(manager_name + "_node")
        child_names = rospy.get_param("/recording/")
        HarmoniRecordingManager(manager_name, child_names)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
