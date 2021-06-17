#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import DialogueNameSpace
from google.api_core.exceptions import InvalidArgument
import dialogflow
import os


class GoogleService(HarmoniServiceManager):
    """
    Google service
    """

    def __init__(self, name, param):
        """Constructor method: Initialization of variables and lex parameters + setting up"""
        super().__init__(name)
        rospy.loginfo("Google initializing")
        self.name = name
        self.project_id = param["project_id"]
        self.language = param["language"]
        self.session_id = param["session_id"]
        self.credential_path = param["credential_path"]
        self.setup_google()
        self.state = State.INIT
        return

    def setup_google(self):
        """ Setup the google request """
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.credential_path
        self.google_client = dialogflow.SessionsClient()
        self.google_session = self.google_client.session_path(
            self.project_id, self.session_id
        )
        return

    def request(self, input_text):
        """[summary]

        Args:
            input_text (str): User request (or input text) for triggering DialogFlow Intent


        Returns:
            object: It containes information about the response received (bool) and response message (str)
                response: bool
                message: str
        """
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        textdata = dialogflow.types.TextInput(
            text=input_text, language_code=self.language
        )
        query_input = dialogflow.types.QueryInput(text=textdata)
        try:
            rospy.loginfo("Request to google")
            google_response = self.google_client.detect_intent(
                session=self.google_session, query_input=query_input
            )

            self.state = State.SUCCESS
            rospy.loginfo(
                "The response is %s" % (google_response.query_result.fulfillment_text)
            )
            self.response_received = True
            self.result_msg = google_response.query_result.fulfillment_text
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
        return {"response": self.response_received, "message": self.result_msg}


def main():
    """[summary]
    Main function for starting HarmoniGoogle service
    """
    service_name = DialogueNameSpace.bot.name
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = AWSLexService(service_id, params)
        service_server = HarmoniServiceServer(service_id, s)
        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
