#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from audio_common_msgs.msg import AudioData
from collections import deque
import numpy as np
import audioop
import pyaudio
import math
import ast
import wget
import os
import wave
import contextlib




class SpeakerService(HarmoniServiceManager):
    """
    Speaker service
    """

    def __init__(self, name, param):
        """ """
        super().__init__(name)

        """ Setup Params """
        self.total_channels = param["total_channels"]
        self.audio_rate = param["audio_rate"]
        self.chunk_size = param["chunk_size"]
        self.device_name = param["device_name"]
        self.output_device_index = None
        """ Setup the speaker """
        # self.p = pyaudio.PyAudio()
        self.audio_format = pyaudio.paInt16
        self.audio_publisher = rospy.Publisher("/audio/audio", AudioData, queue_size=1,)
        """Setup the speaker service as server """
        self.state = State.INIT
        # self.open_stream()
        return

    def do(self, data):
        """ Do the speak """
        self.state = State.REQUEST
        self.actuation_completed = False
        try:
            duration = 0
            if type(data) == str:
                if ".wav" in data:
                    data = self.wav_to_data(data)
                    duration = data["duration"]
                else:
                    data = ast.literal_eval(data)
            data = data["audio_data"]
            rospy.loginfo("Writing data for speaker")
            rospy.loginfo(f"length of data is {len(data)}")
            self.audio_publisher.publish(data)
            rospy.sleep(duration)
            self.state = State.SUCCESS
            self.actuation_completed = True
        except IOError:
            rospy.logwarn("Speaker failed: Audio appears too busy")
            self.state = State.FAILED
            self.actuation_completed = True
        return

    def setup_speaker(self):
        """ Setup the speaker """
        rospy.loginfo("Setting up the %s" % self.name)
        self.get_index_device()
        return

    def open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the audio output stream")
        self.stream.start_stream()
        return

    def close_stream(self):
        """Closing the stream """
        rospy.loginfo("Closing the audio output stream")
        self.stream.stop_stream()
        # self.stream.close()
        # self.p.terminate()
        return

    def get_index_device(self):
        """ 
        Find the output audio devices configured in ~/.asoundrc. 
        If the device is not found, pyaudio will use your machine default device
        """
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            if device["name"] == self.device_name:
                # rospy.loginfo("Found device with name " + self.device_name)
                self.output_device_index = i
                return

    def wav_to_data(self, path):
        """ 
        WAV to audiodata
        """
        file_handle = path
        if "http" in path:
            url = path
            print('Beginning file download with wget module')
            file_handle = '/root/harmoni_catkin_ws/src/HARMONI/harmoni_actuators/harmoni_speaker/temp_data/test.wav'
            wget.download(url, file_handle)
        data = np.fromfile(file_handle, np.uint8)[24:]  # Loading wav file
        data = data.astype(np.uint8).tostring()
        with contextlib.closing(wave.open(file_handle,'r')) as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames / float(rate)
            rospy.loginfo(f"The audio lasts {duration} seconds")
        if "http" in path:
            os.remove(file_handle)
        return {"audio_data": data, "duration": duration}


def main():
    service_name = ActuatorNameSpace.speaker.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        param = rospy.get_param(name + "/" + test_id + "_param/")
        service = hf.set_service_server(service_name, test_id)
        s = SpeakerService(service, param)
        service_server = HarmoniServiceServer(name=service, service_manager=s)
        if test:
            rospy.loginfo("Testing the %s" % (service))
            rospy.sleep(1)
            data = s.wav_to_data(test_input)
            s.audio_publisher.publish(data["audio_data"])
            rospy.loginfo("Testing the %s has been completed!" % (service))
        else:
            service_server.update_feedback()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
