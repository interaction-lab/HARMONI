#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Other Imports
from harmoni_common_lib.constants import SensorNameSpace
from audio_common_msgs.msg import AudioData
from collections import deque
import sys
import pyaudio
import math
import audioop
import wave
import numpy as np
import ast


class MicrophoneService(HarmoniServiceManager):
    """
    Microphone service
    """

    def __init__(self, name, param):
        """ Initialization of variables and microphone parameters """
        super().__init__(name)
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
        self.service_id = hf.get_child_id(self.name)
        """ Setup the microphone """
        self.p = pyaudio.PyAudio()
        self.audio_format = (
            pyaudio.paInt16
        )  # How can we trasform it in a input parameter?
        self.stream = None
        self.setup_microphone()
        """Init the publisher """
        self.mic_pub = rospy.Publisher(
            SensorNameSpace.microphone.value + self.service_id + "/talking",
            AudioData,
            queue_size=1,
        )  # Publishing the voice data
        self.mic_raw_pub = rospy.Publisher(
            SensorNameSpace.microphone.value + self.service_id, AudioData, queue_size=1
        )  # Publishing raw_data
        self.state = State.INIT
        return

    def start(self, rate=""):
        rospy.loginfo("Start the %s service" % self.name)
        if self.state == State.INIT:
            self.state = State.START
            try:
                self.open_stream()
                self.listen()  # Start the microphone service at the INIT
            except Exception:
                self.state = State.FAILED
        else:
            self.state = State.START
        return

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        try:
            self.close_stream()
            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        rospy.sleep(15)
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

    def listen(self):
        """Listening from the microphone """
        rospy.loginfo("The %s is listening" % self.name)
        current_audio = b""
        chunks_per_second = int(self.audio_rate / self.chunk_size)
        sliding_window = deque(maxlen=self.silence_limit_seconds * chunks_per_second)
        prev_audio = deque(maxlen=self.previous_audio_seconds * chunks_per_second)
        started = False
        while not rospy.is_shutdown():
            try:
                if self.state != State.FAILED:
                    latest_audio_data = self.stream.read(
                        self.chunk_size, exception_on_overflow=False
                    )
                    raw_audio_bitstream = np.fromstring(latest_audio_data, np.uint8)
                    raw_audio = raw_audio_bitstream.tolist()
                    self.mic_raw_pub.publish(raw_audio)  # Publishing raw AudioData
                    if self.state == State.START:
                        # Check magnitude of audio
                        sliding_window.append(
                            math.sqrt(
                                abs(
                                    audioop.avg(
                                        latest_audio_data, self.audio_format_width
                                    )
                                )
                            )
                        )
                        Calibrating = False
                        if Calibrating:
                            print_window = str([round(x, 1) for x in sliding_window])
                            maximum = round(max(sliding_window), 2)
                            rospy.loginfo("Noises" + print_window + str(maximum))
                        if any([x > self.silence_threshold for x in sliding_window]):
                            if not started:
                                rospy.loginfo("Sound detected...")
                                started = True
                            current_audio += latest_audio_data
                        elif started:
                            rospy.loginfo("Finished detecting")
                            all_audio_data = b"".join(prev_audio) + current_audio
                            self.state = State.SUCCESS
                            audio_bitstream = np.fromstring(all_audio_data, np.uint8)
                            audio = audio_bitstream.tolist()
                            rospy.loginfo("Publish listening data")
                            self.mic_pub.publish(audio)  # Publishing AudioData of voice
                            started = False
                            sliding_window.clear()
                            prev_audio.clear()
                            current_audio = b""
                            rospy.loginfo("Detection sent. Waiting for new audio...")
                            self.state = State.START
                        else:
                            prev_audio.append(latest_audio_data)
                    else:
                        rospy.loginfo("not listening")
                else:
                    break
            except Exception as e:
                rospy.loginfo("Caught the following exception" + e)

        rospy.loginfo("Shutting down")
        return

    def get_index_device(self):
        """
        Find the input audio devices configured in ~/.asoundrc.
        If the device is not found, pyaudio will use your machine default device
        """
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            rospy.loginfo(device)
            # rospy.loginfo(f"Found device with name {self.device_name} at index {i}")
            if device["name"] == self.device_name:
                rospy.loginfo(device)
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

    def save_data(self):
        """Init the subscriber """
        self.mic_sub = rospy.Subscriber(
            SensorNameSpace.microphone.value + self.service_id,
            AudioData,
            self._record_audio_data_callback,
            queue_size=1,
        )  # Publishing the voice data
        return

    def _record_audio_data_callback(self, data):
        """Callback function to record data"""
        data = np.fromstring(data.data, np.uint8)
        if self.first_audio_frame:
            self.wf = wave.open(self.file_path, "wb")
            self.wf.setnchannels(self.total_channels)
            self.wf.setsampwidth(self.p.get_sample_size(self.audio_format))
            self.wf.setframerate(self.audio_rate)
            self.wf.setnframes(self.chunk_size)
            self.wf.writeframes(b"".join(data))
            self.first_audio_frame = False
        else:
            self.wf.writeframes(b"".join(data))
        return


def main():
    service_name = SensorNameSpace.microphone.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        param = rospy.get_param(name + "/" + test_id + "_param/")
        service = hf.set_service_server(service_name, test_id)
        s = MicrophoneService(service, param)
        service_server = HarmoniServiceServer(name=service, service_manager=s)
        if test:
            rospy.loginfo("Testing the %s" % (service))
            s.save_data()
            s.start()
        else:
            service_server.update_feedback()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
