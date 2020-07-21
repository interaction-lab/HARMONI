#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import io
import os
from google.cloud import speech_v1
from google.cloud.speech_v1 import enums
from harmoni_common_lib.constants import State, RouterDetector
from harmoni_common_lib.helper_functions import HelperFunctions
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
        self.local_path = param["local_path"]
        self.credential_path = param["credential"]
        """ Setup the google request """
        #self.setup_google()
        """Setup the google service as server """
        self.state = State.INIT
        super().__init__(self.state)
        return

    def setup_google(self):
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.credential_path
        self.client = speech_v1.SpeechClient()
        encoding = enums.RecognitionConfig.AudioEncoding.LINEAR16
        self.config = {
            "launguage_code": self.language,
            "sample_rate_hertz":self.sample_rate,
            "encoding": encoding
        }
        return

    def response_update(self, response_received, state, result_msg):
        super().update(response_received=response_received, state=state, result_msg=result_msg)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def request(self, input_text):
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        rate = ""  # TODO: TBD
        super().request(rate)
        #textdata = dialogflow.types.TextInput(text=input_text, language_code=self.language)
        #query_input = dialogflow.types.QueryInput(text=input_text)
        try:
            rospy.loginfo("Request to google")
            #operation = self.client.long_running_recognize(config, audio)
            #response = operation.result()
            #for result in response.results:
            #   alternative = alternatives[0]
            #   response = alternative.transcript
            #self.state = State.SUCCESS
            #rospy.loginfo("The response is %s" % (response)
            #self.response_update(response_received=True, state=self.state, result_msg=response)
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_update(response_received=True, state=self.state, result_msg="")
        return

    def wav_to_data(self):
        with io.open(self.local_path, "rb") as f:
            content = f.read()
        audio_data = content
        return audio_data


def main():
    service_name = RouterDetector.stt.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    input_test = rospy.get_param("/input_test_" + service_name + "/")
    id_test = rospy.get_param("/id_test_" + service_name + "/")
    try:
        service_name = RouterDialogue.bot.name
        rospy.init_node(service_name)
        list_service_names = HelperFunctions.get_child_list(service_name)
        print(list_service_names)
        service_server_list = []
        last_event = ""  # TODO
        for service in list_service_names:
            print(service)
            service_id = HelperFunctions.get_child_id(service)
            param = rospy.get_param(name+"/"+ service_id + "_param/")
            print(param)
            s = STTGoogleService(service, param)
            service_server_list.append(WebServiceServer(name=service, service_manager=s))
            if test and (service_id == id_test):
                rospy.loginfo("Testing the %s" % (service))
                s.request(input_test)
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
