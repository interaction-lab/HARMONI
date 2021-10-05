#!/usr/bin/env python3

# Common Imports
import rospy
import roslib
import threading
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


class WebService(HarmoniServiceManager):
    """
    Web service
    """

    def __init__(self, name):
        """ Initialization of variables and web parameters """
        super().__init__(name)
        self.name = name
        self.service_id = hf.get_child_id(self.name)
        self.is_request = True
        self.web_sub = rospy.Subscriber(
            ActuatorNameSpace.web.value + self.service_id + "/listen_click_event",
            String,
            self._event_click_callback,
            queue_size=1,
        )
        self.web_pub = rospy.Publisher(
            ActuatorNameSpace.web.value + self.service_id + "/set_view",
            String,
            queue_size=1,
        )
        self.text_pub = rospy.Publisher(
            DetectorNameSpace.stt.value + self.service_id, String, queue_size=10
        )
        self.setup_web()
        self.result = None
        self.state = State.INIT
        return

    def setup_web(self):
        """Setup the web service, and waiting for the web browser to connect"""
        rospy.loginfo("Setting up the %s" % self.name)
        rospy.loginfo("Checking that web is connected to ROS websocket")
        rospy.wait_for_service(
            ActuatorNameSpace.web.value + self.service_id + "/is_connected"
        )
        rospy.loginfo("Done, web is connected to ROS websocket")
        return

    def request(self, data):
        """Request to web page: requesting to the webpage to display something and waiting for a user event (e.g., button click, input text)

        Args:
            data (str): string of json which contains two items: {"container_id": str, "set_view": str}
                container_id: id of the container found in the ./web/src/config/config.json file
                set_view: the content you want to set your container of (e.g., string of image location, text)

        Returns:
            response (int): state of the request (SUCCESS, FAIL)
            message (str): content of the response message
        """
        rospy.loginfo("Start the %s do" % self.name)
        self.state = State.REQUEST
        self.actuation_completed = False
        self.result_msg = ""
        self.end_listening = False
        data_array = self._get_web_data(data)

        try:
            while not rospy.is_shutdown() and not self.end_listening:
                rospy.loginfo(f"Waiting for user, the results is: {self.result_msg}")
                if self.end_listening:
                    break
                rospy.sleep(0.1)
            rospy.logdebug(
                f"Message Received {self.result_msg}"
            )  # "\"My name is chris\""
            self.state = State.SUCCESS
            self.actuation_completed = True
            self.response_received = True
            self.end_listening = False
            # def daemon():
            #     while not rospy.is_shutdown() and not self.end_listening:
            #         rospy.loginfo(f"Waiting for user, the results is: {self.result_msg}")
            #         if self.end_listening:
            #             break
            #         rospy.sleep(1)
            #     rospy.logdebug(
            #         f"Message Received {self.result_msg}"
            #     )  # "\"My name is chris\""
            #     self.state = State.SUCCESS
            #     self.actuation_completed = True
            #     self.response_received = True
            #     self.end_listening = False
            # d = threading.Thread(target=daemon)
            # d.start()
        except Exception:
            self.state = State.FAILED
            self.actuation_completed = True
        return {"response": self.state, "message": self.result_msg}

    def do(self, data):
        """Do to web page (display a page on the web browser)

        Args:
            data (str): string of json which contains two items: {"container_id": str, "set_view": str}
                container_id: id of the container found in the ./web/src/config/config.json file
                set_view: the content you want to set your container of (e.g., string of image location, text)

        Returns:
            response (int): state of the request (SUCCESS, FAIL)
            message (str): empty string (not expecting any response)
        """
        rospy.loginfo("Start the %s do" % self.name)
        self.state = State.REQUEST
        self.result_msg = ""
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
        return {"response": self.state, "message": self.result_msg}

    def _get_web_data(self, data):
        """Getting web data from the TTS results

        Args:
            data (str): string of behavior_data object from TTS

        Returns:
            web_array (list): array of items with corresponding values of "container_id" and "set_view" to display when speaking
        """
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
        """Sending the request to the web page

        Args:
            display_view (str): string oj json with information to display ("container_id" and "set_view" values)
        """
        rospy.loginfo("Sending request to webpage")
        print(display_view)
        self.web_pub.publish(display_view)
        return

    def _event_click_callback(self, event):
        """Callback for subscription to the web page"""
        rospy.loginfo("Received an event from the webpage")
        print(type(event.data))
        self.end_listening = True
        # self.result_msg = str(event)[2:-2]
        self.result_msg = event.data
        return

def main():
    """Set names, collect params, and give service to server"""
    service_name = ActuatorNameSpace.web.name
    instance_id = rospy.get_param("/instance_id")
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name)
        # params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = WebService(service_id)
        service_server = HarmoniServiceServer(service_id, s)

        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()