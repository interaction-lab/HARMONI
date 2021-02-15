#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, DetectorNameSpace
from std_msgs.msg import String
import boto3
import json
import ast
from time import time
import threading


class WebService(HarmoniServiceManager):
    """
    Web service
    """

    def __init__(self, name, param):
        """ Initialization of variables and web parameters """
        super().__init__(name)
        self.name = name
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
        self.end_listening = False
        self.web_pub = rospy.Publisher(
            ActuatorNameSpace.web.value + self.service_id + "/set_view",
            String,
            queue_size=1,
        )
        self.text_pub = rospy.Publisher(
            DetectorNameSpace.stt.value + self.service_id, String, queue_size=10
        )
        """ Setup the web request """
        self.setup_web()
        """Setup the web service as server """
        self.state = State.INIT
        return

    def stop(self):
        """ End the web"""
        def daemon():
            self.end_listening = True
            rospy.loginfo("STOP!")
        d = threading.Thread(target=daemon)
        d.setDaemon(True)
        d.start()
        rospy.loginfo("__________END___________")

    def setup_web(self):
        rospy.loginfo("Setting up the %s" % self.name)
        rospy.loginfo("Checking that web is connected to ROS websocket")
        rospy.wait_for_service(
            ActuatorNameSpace.web.value + self.service_id + "/is_connected"
        )
        rospy.loginfo("Done, web is connected to ROS websocket")
        return

    def request(self, data):
        """ Do the display view"""
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        self.actuation_completed = False
        self.result_msg = ""
        data_array = []
        if data!="":
            data_array = self._get_web_data(data)
        try:
            rospy.sleep(0.2)
            for data in data_array:
                self.send_request(data)
                rospy.sleep(0.2)
            def daemon():
                rospy.loginfo("STARTING")
                rospy.loginfo(self.result_msg)
                rospy.loginfo(self.end_listening)
                while not rospy.is_shutdown() and not self.end_listening:
                    rospy.loginfo(f"Waiting for user, the results is: {self.result_msg}")
                    if self.end_listening:
                        break
                    rospy.sleep(1)
                rospy.loginfo(
                    f"Message Received {self.result_msg}"
                )  # "\"My name is chris\""
                self.state = State.SUCCESS
                self.actuation_completed = True
                self.response_received = True
                self.end_listening = False
                rospy.loginfo("ENDING")
            d = threading.Thread(target=daemon)
            d.setDaemon(True)
            d.start()
        except Exception:
            self.state = State.FAILED
            self.actuation_completed = True
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
        except Exception:
            self.state = State.FAILED
            self.actuation_completed = True
        return

    def _get_web_data(self, data):
        web_array = []
        if isinstance(data, str):
            data = ast.literal_eval(data)
        rospy.loginfo("Data received are"+str(data))
        if not isinstance(data, list):
            if "behavior_data" in data.keys():
                if isinstance(data["behavior_data"], str):
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
        print(event.data)
        self.end_listening=True
        if isinstance(event.data, str):
            data = ast.literal_eval(event.data)
        # self.result_msg = str(event)[2:-2]
        if hasattr(data, 'set_view'):
            self.result_msg = data["set_view"]
        elif hasattr(data, 'patient_id'):
            self.result_msg = data["patient_id"]
        else:
            self.result_msg = str(data)
        rospy.loginfo("The result msg is " + self.result_msg)
        return


def main():
    service_name = ActuatorNameSpace.web.name
    name = rospy.get_param("/name_" + service_name + "/")
    test = rospy.get_param("/test_" + service_name + "/")
    test_input = rospy.get_param("/test_input_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        param = rospy.get_param(name + "/" + test_id + "_param/")
        if not hf.check_if_id_exist(service_name, test_id):
            rospy.logerr(
                "ERROR: Remember to add your configuration ID also in the harmoni_core config file"
            )
            return
        service = hf.set_service_server(service_name, test_id)
        s = WebService(service + "_page_" + test_id, param)
        service_server = HarmoniServiceServer(name=service + "_page_" + test_id, service_manager=s)
        if test:
            rospy.loginfo("Testing the %s" % (service))
            rospy.sleep(2)
            s.do(test_input)
        else:
            service_server.update_feedback()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
