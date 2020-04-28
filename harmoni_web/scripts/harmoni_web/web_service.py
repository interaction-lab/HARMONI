#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import boto3
from harmoni_common_lib.constants import State
from harmoni_common_lib.child import WebServiceServer
from harmoni_common_lib.service_manager import HarmoniExternalServiceManager

class WebService(HarmoniExternalServiceManager):
    """
    Web service
    """

    def __init__(self, name, param):
        """ Initialization of variables and web parameters """
        rospy.loginfo("Web initializing")
        self.name = name
        self.user_id = param["user_id"]
        """ Setup the web request """
        self.setup_web()
        """Setup the web service as server """
        self.state = State.INIT 
        super().__init__(self.state)
        return

    def setup_web(self):
        
        return

    def response_update(self, response_received, state, result_msg):
        super().update(response_received=response_received, state = state, result_msg=result_msg)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def request(self, input_text):
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        rate = "" #TODO: TBD
        super().request(rate)
        try:
            
            self.state = State.SUCCESS
            self.response_update(response_received=True, state=self.state, result_msg=lex_response["message"])
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_update(response_received=True, state=self.state, result_msg="")
        return


def main():
    try:
        service_name = "web"
        rospy.init_node(service_name + "_node")
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        param = rospy.get_param("/"+service_name+"_param/")
        s = WebService(service_name, param)
        web_service_server = WebServiceServer(name=service_name, service_manager=s)
        web_service_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
