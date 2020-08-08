#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
from std_msgs.msg import String
import boto3
import json
import ast


class WebService(HarmoniServiceManager):
    """
    Web service
    """

    def __init__(self, name, param):
        """ Initialization of variables and web parameters """
        super().__init__(name)
        self.user_id = param["user_id"]
        self.timer_interval = param["timer_interval"]
        self.service_id = hf.get_child_id(self.name)
        self.is_request = True
        """Setup publisher and subscriber """
        self.web_sub = rospy.Subscriber(
            ActuatorNameSpace.web.value + self.service_id + "/listen_click_event",
            String,
            self._event_click_callback,
            queue_size=1,
        )
        print(ActuatorNameSpace.web.value + self.service_id + "/set_view")
        self.web_pub = rospy.Publisher(
            ActuatorNameSpace.web.value + self.service_id + "/set_view",
            String,
            queue_size=1,
        )
        """ Setup the web request """
        self.setup_web()
        """Setup the web service as server """
        self.state = State.INIT
        return

    def setup_web(self):
        rospy.loginfo("Setting up the %s" % self.name)
        rospy.loginfo("Checking that web is connected to ROS websocket")
        rospy.wait_for_service(
            ActuatorNameSpace.web.value + self.service_id + "/is_connected"
        )
        rospy.loginfo("Done, web is connected to ROS websocket")
        return

    def do(self, data):
        """ Do the display view"""
        rospy.loginfo("Start the %s do" % self.name)
        self.state = State.REQUEST
        self.actuation_completed = False
        data_array = self._get_web_data(data)
        try:
            rospy.sleep(1)
            for data in data_array:
                self.send_request(data)
                rospy.sleep(0.2)
            self.state = State.SUCCESS
            self.actuation_completed = True
        except:
            self.state = State.FAILED
            self.actuation_completed = True
        return

    def _get_web_data(self, data):
        data = ast.literal_eval(data)
        web_array = []
        if not isinstance(data, list):
            if "behavior_data" in data.keys():
                behavior_data = ast.literal_eval(data["behavior_data"])
                for b in behavior_data:
                    if "type" in b.keys():
                        if b["type"] == "web":
                            container_id = b["args"][0]
                            set_view = ""
                            if len(b["args"]) > 1:
                                set_view = b["args"][1]
                            web_array.append(
                                str(
                                    {
                                        "component_id": container_id,
                                        "set_content": set_view,
                                        "start": b["start"],
                                    }
                                )
                            )
            else:
                web_array.append(str(data))
        else:
            for item in data:
                web_array.append(str(item))
        return web_array

    def send_request(self, display_view):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage")
        print(display_view)
        self.web_pub.publish(display_view)
        return

    def _event_click_callback(self, event):
        """Callback for subscription to the web page"""
        rospy.loginfo("Received an event from the webpage")
        return


def main():
    service_name = ActuatorNameSpace.web.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        last_event = ""  # TODO: How to get information about last_event from behavior controller?
        list_service_names = hf.get_child_list(service_name)
        service_server_list = []
        for service in list_service_names:
            rospy.loginfo(service)
            service_id = hf.get_child_id(service)
            param = rospy.get_param(name + "/" + service_id + "_param/")
            s = WebService(service, param)
            service_server_list.append(
                HarmoniServiceServer(name=service, service_manager=s)
            )
            if test and (service_id == test_id):
                rospy.loginfo("Testing the %s" % (service))
                rospy.sleep(3)
                s.do(test_input)
        if not test:
            for server in service_server_list:
                server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
