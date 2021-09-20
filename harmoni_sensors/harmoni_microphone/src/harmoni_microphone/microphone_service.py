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
import pyaudio
import wave
import numpy as np


class MicrophoneService(HarmoniServiceManager):
    """Reads from a microphone and publishes audio data.

    As a sensor service, the microphone is responsible for reading the audio data
    from a physical microphone and publishing it so that it can be recorded or
    transcribed by a detector.

    The microphone has many parameters which are set in the configuration.yaml

    The public functions exposed by the microphone include start(), stop(), and pause()
    """

    def __init__(self, name, param):

        """ Initialization of variables and microphone parameters """
        super().__init__(name)
        self.audio_format_width = param["audio_format_width"]
        self.chunk_size = param["chunk_size"]
        self.total_channels = param["total_channels"]
        self.audio_rate = param["audio_rate"]
        self.device_name = param["device_name"]
        self.file_path = param["test_outdir"]

        self.first_audio_frame = True  # When recording, the first frame is special

        self.service_id = hf.get_child_id(self.name)

        """ Setup the microphone """
        self.p = pyaudio.PyAudio()

        # TODO: How can we trasform audio_format as an a input parameter?/should we?
        self.audio_format = pyaudio.paInt16
        self.stream = None
        self.setup_microphone()

        """ Init the publishers """
        self.microphone_topic = SensorNameSpace.microphone.value + self.service_id
        self.raw_mic_pub = rospy.Publisher(
            self.microphone_topic, AudioData, queue_size=1
        )

        self.state = State.INIT
        return

    def start(self):
        """Start the microphone stream and publish audio"""
        rospy.loginfo("Start the %s service" % self.name)
        if (
            self.state == State.INIT
            or self.state == State.FAILED
            or self.state == State.SUCCESS
        ):
            self.state = State.START
            self._open_stream()  # TODO handle possible exception from this
            self._read_stream_and_publish()
            self.results_msg = ""
            self.state = State.SUCCESS
        elif self.state == State.PAUSE:
            self.state = State.START
            self._read_stream_and_publish()
            self.state = State.SUCCESS
        else:
            rospy.loginfo("Trying to start stream when already started")
            self.state = State.START
        return

    def stop(self):
        """Stop the service and close the stream"""
        rospy.loginfo("Stop the %s service" % self.name)
        try:
            self._close_stream()
            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        """Set the service to success to stop publishing"""
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.PAUSE
        return

    def setup_microphone(self):
        """ Setup the microphone """
        rospy.loginfo("Setting up the %s" % self.name)
        self._get_device_index()  # get index of the input audio device
        return

    def _open_stream(self):
        """Open the microphone audio stream with configured params """
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

    def _close_stream(self):
        """ When possibly, the pyaudio stream object should be closed """
        self.stream.stop_stream()
        self.stream.close()
        return

    def _read_stream_and_publish(self):
        """Continously publish audio data from the microphone

        While state is START publish audio
        """
        r = rospy.Rate(10)
        rospy.loginfo("The %s is listening" % self.name)
        while not rospy.is_shutdown():
            if self.state == State.INIT:
                r.sleep()
            elif self.state == State.START:
                latest_audio_data = self.stream.read(
                    self.chunk_size, exception_on_overflow=False
                )
                raw_audio_bitstream = np.frombuffer(latest_audio_data, np.uint8)
                raw_audio = raw_audio_bitstream.tolist()
                self.raw_mic_pub.publish(raw_audio)  # Publishing raw AudioData
            elif (
                self.state == State.SUCCESS
                or self.state == State.FAILED
                or self.state == State.PAUSE
            ):
                break
            r.sleep()

        rospy.loginfo("Shutting down")
        return

    def _get_device_index(self):
        """
        Find the input audio devices configured in ~/.asoundrc.
        If the device is not found, pyaudio will use your machine default device
        """
        self.input_device_index = 0
        for i in range(self.p.get_device_count()):
            device = self.p.get_device_info_by_index(i)
            # rospy.loginfo(device)
            # rospy.loginfo(f"Found device with name {self.device_name} at index {i}")
            if device["name"] == self.device_name:
                rospy.loginfo(device)
                self.input_device_index = i
        return

    def start_recording_data(self):
        """Init the subscriber to microphone/default for recording audio.

        The callback in the subscriber will save the audio to a file
        specified in the configuration yaml.
        """
        rospy.loginfo(f"Start recording to {self.file_path}")

        self.mic_sub = rospy.Subscriber(
            self.microphone_topic,
            AudioData,
            self._record_audio_data_callback,
            queue_size=1,
        )
        return

    def _record_audio_data_callback(self, data):
        """Callback function to write data"""
        data = np.fromstring(data.data, np.uint8)
        if self.first_audio_frame:
            rospy.loginfo("Start recording")
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
    """Set names, collect params, and give service to server"""

    service_name = SensorNameSpace.microphone.name  # "microphone"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name)

        # microphone/default_param/[all your params]
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = MicrophoneService(service_id, params)

        service_server = HarmoniServiceServer(service_id, s)

        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        s.start()

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
