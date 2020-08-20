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
        super().__init__(name)
        """ Initialization of variables and google parameters """
        rospy.loginfo("Google initializing")
        self.name = name
        self.project_id = param["project_id"]
        self.language = param["language"]
        self.session_id = param["session_id"]
        self.credential_path = param["credential_path"]
        """ Setup the google request """
        self.setup_google()
        """Setup the google service as server """
        self.state = State.INIT
        return

    def setup_google(self):
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = self.credential_path
        self.google_client = dialogflow.SessionsClient()
        self.google_session = self.google_client.session_path(
            self.project_id, self.session_id
        )
        return

    def request(self, input_text):
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
            rospy.loginfo("The response is %s" % (google_response.query_result.fulfillment_text))
            self.response_received = True
            self.result_msg = google_response.query_result.fulfillment_text
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
        return


def main():
    service_name = DialogueNameSpace.bot.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        param = rospy.get_param(name + "/" + test_id + "_param/")
        if not hf.check_if_id_exist(service_name, test_id):
            rospy.logerr("ERROR: Remember to add your configuration ID also in the harmoni_core config file")
            return
        service = hf.set_service_server(service_name, test_id)
        s = GoogleService(service, param)
        service_server = HarmoniServiceServer(name=service, service_manager=s)
        if test:
            rospy.loginfo("Testing the %s" % (service))
            s.request(test_input)
        else:
            service_server.update_feedback()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
