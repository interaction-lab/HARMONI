#!/usr/bin/env python3

#Common Imports
import rospy
import roslib 

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import State, DetectorNameSpace, SensorNameSpace
from audio_common_msgs.msg import AudioData
from google.cloud import speech
from std_msgs.msg import String
import numpy as np
import os
import io


class SpeechToTextService(HarmoniServiceManager):
    """
    Speech to text service using DeepSpeech
    """

    def __init__(self, name, param):
        """ Initialization of variables and google parameters """
        super().__init__(name)
        self.sample_rate = param["sample_rate"]
        self.language = param["language_id"]
        self.audio_channel = param["audio_channel"]
        self.credential_path = param["credential_path"]
        self.subscriber_id = param["subscriber_id"]

        self.service_id = hf.get_child_id(self.name)

        

        """Setup publishers and subscribers"""
        rospy.Subscriber("/audio/audio", AudioData, self.playing_sound_pause_callback)
        rospy.Subscriber(
            SensorNameSpace.microphone.value + self.subscriber_id,
            AudioData,
            self.sound_data_callback,
        )
        self.text_pub = rospy.Publisher(
            DetectorNameSpace.stt.value + self.service_id, String, queue_size=10
        )

        self.state = State.INIT
        return
