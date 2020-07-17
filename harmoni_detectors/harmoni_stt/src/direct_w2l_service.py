#!/usr/bin/env python3.6
# NEW
# Importing the libraries
from subprocess import Popen, PIPE
import select
import pty
import os
import time
import re
import rospy
import roslib
import pyaudio
import math
import audioop
import wave
import numpy as np
import ast
from collections import deque
from harmoni_common_lib.constants import State, RouterDetector, RouterSensor
from harmoni_common_lib.helper_functions import HelperFunctions
from harmoni_common_lib.child import InternalServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from multiprocessing import Process
import multiprocessing as mp


class SpeechToTextService(HarmoniServiceManager):
    """
    Speech to text service using wave2letter
    """

    def __init__(self, name, param):

        """ Initialization of variables and w2l parameters """
        rospy.loginfo("Wav2Letter initializing")
        self.name = name
        self.subscriber_id = param["subscriber_id"]
        self.model_path = param["model_path"]
        if not os.path.isdir(self.model_path):
            raise Exception(
                "W2L model has not been dowloaded", "Try running get_w2l_models.sh"
            )
        self.w2l_bin = param["w2l_bin"]
        self.service_id = HelperFunctions.get_child_id(self.name)

        """Setup publishers and subscribers"""
        self.text_pub = rospy.Publisher(
            RouterDetector.stt.value + self.service_id, String, queue_size=10
        )
        """Setup the stt service as server """
        self.state = State.INIT
        super().__init__(self.state)

        self.audio_format_width = param["audio_format_width"]
        self.chunk_size = param["chunk_size"]
        self.total_channels = param["total_channels"]
        self.audio_rate = param["audio_rate"]
        self.silence_limit_seconds = param["silence_limit_seconds"]
        self.previous_audio_seconds = param["previous_audio_seconds"]
        self.total_silence_samples = param["total_silence_samples"]
        self.silence_threshold = param["silence_threshold"]
        self.loudest_sound_value = param["loudest_sound_value"]
        self.device_name = param["device_name"]
        self.set_threshold = param["set_threshold"]
        self.file_path = param["test_outdir"]
        self.first_audio_frame = True
        self.service_id = HelperFunctions.get_child_id(self.name)
        """ Setup the microphone """
        self.p = pyaudio.PyAudio()
        self.audio_format = (
            pyaudio.paInt16
        )  # How can we trasform it in a input parameter?
        self.stream = None
        self.setup_microphone()

        return

    def state_update(self):
        super().update(self.state)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def start(self, rate=""):
        rospy.loginfo("Start the %s service" % self.name)
        super().start(rate)
        if self.state == State.INIT:
            self.state = State.START
            self.state_update()
            self.open_stream()
            self.transcribe_stream()  # Start the microphone service at the INIT
            self.state = State.FAILED
        else:
            self.state = State.START
        # self.state_update()
        return

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        super().stop()
        try:
            self.close_stream()
            self.state = State.SUCCESS
            self.state_update()
        except:
            self.state = State.FAILED
            self.state_update()
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        super().pause()
        self.state = State.SUCCESS
        self.state_update()
        return

    def setup_microphone(self):
        """ Setup the microphone """
        rospy.loginfo("Setting up the %s" % self.name)
        self.get_index_device()  # get index of the input audio device
        self.determine_silence_threshold(self.set_threshold)
        return

    def open_stream(self):
        """Opening the stream """
        rospy.loginfo("Opening the audio input stream")
        self.stream = self.p.open(
            format=self.audio_format,
            channels=self.total_channels,
            rate=self.audio_rate,
            input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=self.chunk_size,
        )
        return

    def close_stream(self):
        """Closing the stream """
        self.stream.stop_stream()
        self.stream.close()
        return

    def get_index_device(self):
        """ 
        Find the input audio devices configured in ~/.asoundrc. 
        If the device is not found, pyaudio will use your machine default device
        """
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            print(device)
            rospy.loginfo(f"Found device with name {self.device_name} at index {i}")
            if device["name"] == self.device_name:
                print(device)
                self.input_device_index = i
        return

    def determine_silence_threshold(self, mode):
        """Determine silence threshold from the mic or setting a constant value """
        loudest_sound_cohort_size = 0.2
        silence_threshold_multiplier = 5
        if mode == "default":
            self.open_stream()
            tss = self.total_silence_samples
            values = [
                math.sqrt(
                    abs(
                        audioop.avg(
                            self.stream.read(self.chunk_size), self.audio_format_width
                        )
                    )
                )
                for _ in range(tss)
            ]
            values = sorted(values, reverse=True)
            sum_of_loudest_sounds = sum(values[: int(tss * loudest_sound_cohort_size)])
            total_samples_in_cohort = int(tss * loudest_sound_cohort_size)
            average_of_loudest_sounds = sum_of_loudest_sounds / total_samples_in_cohort
            self.close_stream()
        elif mode == "constant":
            average_of_loudest_sounds = self.loudest_sound_value
        rospy.loginfo("Average audio intensity is " + str(average_of_loudest_sounds))
        self.silence_threshold = (
            average_of_loudest_sounds * silence_threshold_multiplier
        )
        rospy.loginfo("Silence threshold set to " + str(self.silence_threshold))
        return

    def write_to_w2l(self, w2l_process):
        while not rospy.is_shutdown():
            latest_audio_data = self.stream.read(
                self.chunk_size, exception_on_overflow=False
            )
            raw_audio_bitstream = np.fromstring(latest_audio_data, np.uint8)
            raw_audio = raw_audio_bitstream.tolist()

            temp = np.asarray(raw_audio, dtype=np.byte)

            w2l_process.stdin.write(temp)
            w2l_process.stdin.flush()
        return

    def transcribe_stream(self):
        rospy.loginfo("Openning up W2L process")
        w2l_process = Popen(
            ["{} --input_files_base_path={}".format(self.w2l_bin, self.model_path)],
            stdin=PIPE,
            stdout=PIPE,
            stderr=PIPE,
            shell=True,
        )
        """Listening from the microphone """
        rospy.loginfo("Setting up transcription")
        for i in range(17):
            output = w2l_process.stdout.readline()
        p = mp.Process(target=self.write_to_w2l, args=(w2l_process,))
        p.start()
        total_text = ""
        rospy.loginfo("Setup complete")
        while not rospy.is_shutdown():
            output = w2l_process.stdout.readline()
            text = self.fix_text(output)
            if text:
                total_text = total_text + " " + text
            else:
                if total_text:
                    rospy.loginfo("Heard:" + total_text)
                    self.text_pub.publish(total_text[1:])
                    total_text = ""
        p.join()

    def fix_text(self, output):
        text = output.decode("utf-8")
        text = text.split(",")[2][:-2]
        # Remove some bad outputs
        if len(text) > 0:
            if text[0] == "h" and len(text) == 1:
                text = ""
        if len(text) > 1:
            if text[:2] == "h " or text == " transcriptio":
                text = ""
        return text


def main():
    service_name = RouterDetector.stt.name
    name = rospy.get_param("/name_" + service_name + "/")

    test = rospy.get_param("/test_" + service_name + "/")
    input_test = rospy.get_param("/input_test_" + service_name + "/")
    id_test = rospy.get_param("/id_test_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        list_service_names = HelperFunctions.get_child_list(service_name)
        service_server_list = []
        for service in list_service_names:
            print(service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param(name + "/" + service_id + "_param/")
            s = SpeechToTextService(service, param)
            service_server_list.append(
                InternalServiceServer(name=service, service_manager=s)
            )
            if test and (service_id == id_test):
                rospy.loginfo("Testing the %s" % (service))
                s.start()
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
