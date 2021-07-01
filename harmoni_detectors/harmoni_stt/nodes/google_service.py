#!/usr/bin/env python3

# Common Imports
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
    Speech to text service using Google service
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

        """ Setup the google request """
        self.setup_google()

        """Setup the google service as server """
        self.response_text = ""
        self.data = b""

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

    def start(self, rate=""):
        rospy.loginfo("Start the %s service" % self.name)
        if self.state == State.INIT:
            self.state = State.START
            # self.transcribe_stream()  # Start the microphone service at the INIT
        else:
            self.state = State.START
        return

    def stop(self):
        rospy.loginfo("Stop the %s service" % self.name)
        try:
            # self.close_stream()
            self.state = State.SUCCESS
        except Exception:
            self.state = State.FAILED
        return

    def pause(self):
        rospy.loginfo("Pause the %s service" % self.name)
        self.state = State.SUCCESS
        return

    def setup_google(self):
        """Setup google client for speech recognition"""
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.credential_path
        self.client = speech.SpeechClient()
        encoding = speech.RecognitionConfig.AudioEncoding.LINEAR16
        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.sample_rate,
            language_code=self.language,
            audio_channel_count=self.audio_channel,
        )
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.sample_rate,
            language_code=self.language,
        )
        self.streaming_config = speech.StreamingRecognitionConfig(
            config=config, interim_results=True
        )
        return

    def sound_data_callback(self, data):
        """ Callback function subscribing to the microphone topic"""
        data = np.fromstring(data.data, np.uint8)
        # self.data = self.data.join(data)
        # self.data = data.data
        if self.state == State.START:
            self.transcribe_stream_request(self.data)
        else:
            rospy.loginfo(f"Not Transcribing data because state is {self.state}")

    def transcribe_stream_request(self, data):
        # TODO: streaming transcription https://github.com/googleapis/python-speech/blob/master/samples/microphone/transcribe_streaming_infinite.py
        stream = data
        rospy.loginfo("Transcribing Stream")
        requests = (
            speech.StreamingRecognizeRequest(audio_content=chunk) for chunk in stream
        )
        responses = self.client.streaming_recognize(
            config=self.streaming_config, requests=requests
        )
        rospy.loginfo(f"Responses: {responses}")
        for response in responses:
            rospy.loginfo(f"Response items: {response}")
            # Once the transcription has settled, the first result will contain the
            # is_final result. The other results will be for subsequent portions of
            # the audio.
            for result in response.results:
                print("Finished: {}".format(result.is_final))
                print("Stability: {}".format(result.stability))
                alternatives = result.alternatives
                # The alternatives are ordered from most likely to least.
                for alternative in alternatives:
                    print("Confidence: {}".format(alternative.confidence))
                    print("Transcript: {}".format(alternative.transcript))
        return

    def transcribe_file_request(self, data):
        rate = ""  # TODO: TBD
        audio = {"content": data}
        try:
            rospy.loginfo("Request to google")
            operation = self.client.long_running_recognize(
                config=self.config, audio=audio
            )
            rospy.loginfo("Waiting for the operation to complete.")
            self.state = State.PAUSE
            response = operation.result()
            for result in response.results:
                self.data = b""
                alternative = result.alternatives[0]
                text = alternative.transcript
                rospy.loginfo("The response is %s" % (text))
                print(self.response_text)
                if text:
                    self.response_text = self.response_text + " " + text
                else:
                    if self.response_text:
                        rospy.loginfo("Heard:" + self.response_text)
                        self.text_pub.publish(self.response_text[1:])
                        self.response_text = ""
            self.state = State.START
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""
        return

    def request(self, input_data):
        self.data = self.data.join(input_data)
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.START
        self.transcribe_stream_request(self.data)
        return

    def playing_sound_pause_callback(self, data):
        """Sleeps when data is being published to the speaker"""
        rospy.loginfo(f"pausing for data: {len(data.data)}")
        self.pause()
        rospy.sleep(int(len(data.data) / 22040))
        self.start()
        return


def wav_to_data(path):
    with io.open(path, "rb") as f:
        content = f.read()
    return content


def main():
    """Set names, collect params, and give service to server"""

    service_name = DetectorNameSpace.stt.name  # "stt"
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"

    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)

        # stt/default_param/[all your params]
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        s = SpeechToTextService(service_id, params)

        service_server = HarmoniServiceServer(service_id, s)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()