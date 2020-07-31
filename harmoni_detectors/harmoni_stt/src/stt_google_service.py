#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import numpy as np
import os
import io
from google.cloud import speech_v1
from google.cloud.speech_v1 import enums
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from harmoni_common_lib.constants import State, RouterDetector, RouterSensor
import harmoni_common_lib.helper_functions as hf
from harmoni_common_lib.child import WebServiceServer
from harmoni_common_lib.service_manager import HarmoniExternalServiceManager


class STTGoogleService(HarmoniExternalServiceManager):
    """
    Google service
    """

    def __init__(self, name, param):
        """ Initialization of variables and google parameters """
        rospy.loginfo("Google initializing")
        self.name = name
        self.sample_rate = param["sample_rate"]
        self.language = param["language_id"]
        self.audio_channel = param["audio_channel"]
        self.credential_path = param["credential_path"]
        self.subscriber_id = param["subscriber_id"]
        self.service_id = hf.get_child_id(self.name)
        """ Setup the google request """
        self.setup_google()
        """Setup the google service as server """
        self.state = State.INIT
        super().__init__(self.state)
        self.response_text = ""
        self.data = b""
        """Setup publishers and subscribers"""
        rospy.Subscriber(
            RouterSensor.microphone.value + self.subscriber_id,
            AudioData,
            self.callback,
        )
        rospy.Subscriber("/audio/audio", AudioData, self.pause_back)
        self.text_pub = rospy.Publisher(
            RouterDetector.stt.value + self.service_id, String, queue_size=10
        )
        """Setup the stt service as server """
        self.state = State.INIT
        super().__init__(self.state)
        return

    def pause_back(self, data):
        rospy.loginfo(f"pausing for data: {len(data.data)}")
        self.pause()
        rospy.sleep(int(len(data.data) / 30000))  # TODO calibrate this guess
        self.state = State.START
        self.state_update()
        return

    def setup_google(self):
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.credential_path
        self.client = speech_v1.SpeechClient()
        encoding = enums.RecognitionConfig.AudioEncoding.LINEAR16
        self.config = {
            "language_code": self.language,
            "sample_rate_hertz": self.sample_rate,
            "encoding": encoding,
            "audio_channel_count": self.audio_channel,
        }
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=language_code,
        )
        self.streaming_config = types.StreamingRecognitionConfig(
            config=config, interim_results=True
        )
        return

    def response_update(self, response_received, state, result_msg):
        super().update(
            response_received=response_received, state=state, result_msg=result_msg
        )
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def callback(self, data):
        """ Callback function subscribing to the microphone topic"""
        # data = np.fromstring(data.data, np.uint8)
        self.data = self.data.join(data)
        if self.state == State.START:
            self.transcribe_stream_request(self.data)
        else:
            rospy.loginfo("Not Transcribing data")

    def transcribe_stream_request(self, data):
        # TODO: streaming transcription
        requests = (
            types.StreamingRecognizeRequest(audio_content=content)
            for content in audio_generator
        )
        responses = client.streaming_recognize(streaming_config, requests)
        return

    def transcribe_file_request(self, data):
        rate = ""  # TODO: TBD
        super().request(rate)
        audio = {"content": data}
        try:
            rospy.loginfo("Request to google")
            operation = self.client.long_running_recognize(self.config, audio)
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
            self.response_update(
                response_received=True, state=self.state, result_msg=""
            )
        return

    def request(self, input_data):
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.START
        return

    def wav_to_data(self, path):
        with io.open(path, "rb") as f:
            content = f.read()
        return content


def main():
    service_name = RouterDetector.stt.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    input_test = rospy.get_param("/input_test_" + service_name + "/")
    id_test = rospy.get_param("/id_test_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        list_service_names = hf.get_child_list(service_name)
        print(list_service_names)
        service_server_list = []
        last_event = ""  # TODO
        for service in list_service_names:
            rospy.loginfo(service)
            service_id = hf.get_child_id(service)
            param = rospy.get_param(name + "/" + service_id + "_param/")
            print(param)
            s = STTGoogleService(service, param)
            service_server_list.append(
                WebServiceServer(name=service, service_manager=s)
            )
            if test and (service_id == id_test):
                rospy.loginfo("Testing the %s" % (service))
                data = s.wav_to_data(input_test)
                s.request(data)
                s.transcribe_file_request(data)
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
