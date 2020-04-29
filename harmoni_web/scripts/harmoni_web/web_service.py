#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import boto3
from std_msgs.msg import String
from harmoni_common_lib.constants import State
from harmoni_common_lib.child import WebServiceServer, HardwareControlServer
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
        self.is_request = True
        """ Setup the web request """
        self.setup_web()
        """Setup publisher and subscriber """
        self.web_sub = rospy.Subscriber("harmoni/web/listen_click_event", String, self._event_click_callback, queue_size=1)
        self.web_pub = rospy.Publisher("harmoni/web/display_view", String, queue_size=1)
        """Setup the web service as server """
        self.state = State.INIT 
        super().__init__(self.state)
        return

    def setup_web(self):
        rospy.loginfo("Setting up the %s" % self.name)
        rospy.loginfo("Checking that web is connected to ROS websocket")
        rospy.wait_for_service("/harmoni/web/is_connected")
        rospy.loginfo("Done, web is connected to ROS websocket")
        return

    def response_update(self, response_received, state, result_msg):
        super().update(response_received=response_received, state = state, result_msg=result_msg)
        return

    def actuation_update(self, actuation_completed):
        """Update the actuation state """
        rospy.loginfo("Update face state")
        super().update(state = self.state, actuation_completed=actuation_completed)
        return

    def test(self):
        super().test()
        rospy.loginfo("Test the %s service" % self.name)
        success = True
        return success

    def request(self, display_view):
        rospy.loginfo("Start the %s request" % self.name)
        self.is_request = True
        self.state = State.REQUEST
        rate = "" #TODO: TBD
        super().request(rate)
        try:
            self._send_request(display_view)
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_update(response_received=True, state=self.state, result_msg="")
        return

    def do(self, data):
        """ Do the display view"""
        rospy.loginfo("Start the %s do" % self.name)
        self.is_request = False
        data = super().do(data)
        self.state = State.REQUEST
        self.actuation_update(actuation_completed = False)
        try:    
            self._send_request(display_view)
            self.state = State.SUCCESS
            self.actuation_update(actuation_completed = True)
        except:
            self.state = State.FAILED
            self.actuation_update(actuation_completed = True)
        return

    def _send_request(self, display_view):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage")
        self.web_pub.publish(display_view)
        return

    def _event_click_callback(self, event):
        """Callback for subscription to the web page"""
        rospy.loginfo("Received an event from the webpage")
        if self.state == State.REQUEST and self.is_request:
            self.state = State.SUCCESS
            self.response_update(response_received=True, state=self.state, result_msg=event.data)
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
        web_service_server_actuator = HardwareControlServer(name=service_name+"_display", service_manager=s)
        web_service_server_actuator.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
