#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State, SensorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

import pyaudio
import wave
import sys
from datetime import datetime
import numpy as np

path = sys.path
using_kinetic = any([True for p in path if ("kinetic" in p)])
if using_kinetic:
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
    sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2
from cv_bridge import CvBridge, CvBridgeError

# import ffmpeg
from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import Image


class RecordingManager(HarmoniServiceManager):
    """
    The recording manager aims at storing the information collected by the sensors
    TODO: fix the topic names
    """

    def __init__(self, name, child_names):
        """ Init recording manager"""
        rospy.loginfo("Init recording the data from the sensors")
        super().__init__(name)
        self.outdir = child_names["outdir"]
        self.audio_child = child_names["audio_data"]
        self.video_child = child_names["video_data"]
        self.merge_child = child_names[
            "audio_video_data"
        ]  # it returns an array: [0]: audio child, [1]: video child
        self.audio_children = {}
        self.video_children = {}
        self.path_audio = {}
        self.path_video = {}
        self.wf = {}
        self.video_data = {}
        self.frame = {}
        format_size = pyaudio.paInt16
        self.cv_bridge = CvBridge()
        """ Init subscribers"""
        for child in self.audio_child:
            child_id = hf.get_child_id(child)
            child_name = hf.get_service_name(child)
            try:
                param = rospy.get_param(child_name + "/" + child_id + "_param")
            except:
                rospy.logerr("ERR: Remember to run the microphone you want to record")
                return
            self.audio_children[child] = {
                "first_frame": True,
                "channels": param["total_channels"],
                "chunk_size": param["chunk_size"],
                "sample_rate": param["audio_rate"],
                "format_size": format_size,
            }
            if child == self.merge_child[0]:
                rospy.Subscriber(
                    SensorNameSpace.microphone.value + child_id + "/talking",
                    AudioData,
                    self._audio_merge_data_callback,
                    child,
                    queue_size=1,
                )
            else:
                rospy.Subscriber(
                    SensorNameSpace.microphone.value + child_id + "/talking",
                    AudioData,
                    self._audio_data_callback,
                    child,
                    queue_size=1,
                )
        for child in self.video_child:
            child_id = hf.get_child_id(child)
            child_name = hf.get_service_name(child)
            try:
                param = rospy.get_param(child_name + "/" + child_id + "_param")
            except:
                rospy.logerr("ERR: Remember to run the camera you want to record")
                return
            self.video_children[child] = {
                "first_frame": True,
                "video_format": param["video_format"],
                "fps": param["fps"],
            }
            if child == self.merge_child[1]:
                rospy.Subscriber(
                    SensorNameSpace.camera.value + child_id + "/watching",
                    Image,
                    self._video_merge_data_callback,
                    child,
                    queue_size=1,
                )
            else:
                rospy.Subscriber(
                    SensorNameSpace.camera.value + child_id + "/watching",
                    Image,
                    self._video_data_callback,
                    child,
                    queue_size=1,
                )
        self.state = State.INIT

    def start(self):
        self.state = State.START
        rospy.loginfo(f"Starting {self.name}")
        return

    def _record_audio(self, data, child):
        """Record audio file"""
        rospy.loginfo("Recording audio file")
        data = np.fromstring(data.data, np.uint8)
        if self.audio_children[child]["first_frame"]:
            file_name_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            p = pyaudio.PyAudio()
            file_name = "audio_" + file_name_date
            self.path_audio[child] = self.outdir + file_name + ".wav"
            self.wf[child] = wave.open(self.path_audio[child], "wb")
            self.wf[child].setnchannels(self.audio_children[child]["channels"])
            self.wf[child].setsampwidth(
                p.get_sample_size(self.audio_children[child]["format_size"])
            )
            self.wf[child].setframerate(self.audio_children[child]["sample_rate"])
            self.wf[child].setnframes(self.audio_children[child]["chunk_size"])
            self.wf[child].writeframes(b"".join(data))
            self.audio_children[child]["first_frame"] = False
        else:
            self.wf[child].writeframes(b"".join(data))
        return self.path_audio[child]

    def _record_video(self, data, child):
        """Record vide file"""
        rospy.loginfo("Recording video file")
        if self.video_children[child]["first_frame"]:
            file_name_date = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            file_name = "video_" + file_name_date
            self.path_video[child] = self.outdir + file_name + ".avi"
            self.video_data[child] = cv2.VideoWriter(
                self.path_video[child],
                fourcc,
                self.video_children[child]["fps"],
                (data.width, data.height),
                True,
            )
            self.video_children[child]["first_frame"] = False
        else:
            self.frame[child] = self.cv_bridge.imgmsg_to_cv2(
                data, self.video_children[child]["video_format"]
            )
        try:
            self.video_data[child].write(self.frame[child])
        except Exception:
            rospy.logdebug("Not writing!!")
        return self.path_video[child]

    def _audio_data_callback(self, data, child):
        """ Do something when audio data has been received """
        rospy.loginfo("The audio data received from %s" % child)
        self._record_audio(data, child)
        return

    def _video_data_callback(self, data, child):
        """ Do something when video data has been received """
        rospy.logdebug("The video data received from %s" % child)
        self._record_video(data, child)
        return

    def _audio_merge_data_callback(self, data, child):
        """ Do something when audio data has been received """
        rospy.loginfo("The audio data received from %s" % child)
        self.path_audio = self._record_audio(data, child)
        return

    def _video_merge_data_callback(self, data, child):
        """ Do something when video data has been received """
        rospy.logdebug("The video data received from %s" % child)
        self.path_video = self._record_video(data, child)
        return

    def _merge_audio_video(self):
        """Merges audio and video in a single file"""
        input_video = ffmpeg.input(self.path_video)
        added_audio = ffmpeg.input(self.path_audio)
        (
            ffmpeg.concat(input_video, added_audio, v=1, a=1)
            .output(self.outdir + "mix_delayed_audio.mp4")
            .run(overwrite_output=True)
        )
        return


def main():
    name = rospy.get_param("/unit_name/")
    test = rospy.get_param("/test_" + name + "/")
    test_input = rospy.get_param("/test_input_" + name + "/")
    instance_id = rospy.get_param("/instance_id_" + name + "/")
    child_names = rospy.get_param("/" + name + "/")
    try:
        rospy.init_node(name)
        # Initialize the pattern with pattern sequence/loop
        rm = RecordingManager(name, child_names)
        service_server = HarmoniServiceServer(name=name, service_manager=rm)
        if test:
            rospy.loginfo(f"START: Set up. Testing first step of {name} pattern.")
            rm.start()
        else:
            service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
