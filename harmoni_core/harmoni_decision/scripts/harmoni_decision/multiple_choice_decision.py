#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
import rospkg
import json
import inspect
import ast
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, ActionType, ActuatorNameSpace
from collections import deque
from time import time
import threading

class MultipleChoiceDecisionManager(HarmoniServiceManager):
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, name, script, test_id, path, url):
        super().__init__(name)
        self.name = name
        self.script = script
        self.url = url
        self.service_id  = test_id
        self.pattern_script_path = path
        self.index = 0
        self.max_index = 18
        self.choice_index = 16
        self.sequence_scenes = []
        self.scripted_services = ["multiple_choice"] #get the json names
        self._setup_clients()
        self.setup_scene()
        self.state = State.INIT
        

    def _setup_clients(self):
        """
        Set up the pattern client
        """
        for client in self.scripted_services:
            rospy.loginfo(client)
            self.service_clients[client] = HarmoniActionClient(client)
            self.client_results[client] = deque()
        rospy.loginfo("Clients created")
        rospy.loginfo(
            f"{self.name} Decision manager needs these services: {self.scripted_services}"
        )

        for cl, client in self.service_clients.items():
            client.setup_client(cl, self._result_callback, self._feedback_callback)
        rospy.loginfo("Decision interface action clients have been set up!")
        return


    def start(self):
        optional_data = "TRY"
        self.state = State.START
        for service in self.scripted_services:
            service = "multiple_choice"
            rospy.loginfo(service)
            self.service_clients[service].send_goal(
                        action_goal=ActionType.REQUEST,
                        optional_data=optional_data,
                        wait=True,
                    )
            rospy.loginfo(f"Goal sent to {service}")
        return

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_results[result["service"]].append(
            {"time": time(), "data": result["message"]}
        )
        result_data = ast.literal_eval(result["message"])
        for data in result_data:
            if "w" in data:
                web_result = data["w"]
        if "Target" in web_result:
            # send to next
        # TODO add handling of errors and continue=False
        return

    def _feedback_callback(self, feedback):
        """ Send the feedback state to the Behavior Pattern tree to decide what to do next """
        rospy.logdebug("The feedback recieved is %s and nothing more" % feedback)
        # Check if the state is end, stop the behavior pattern
        # if feedback["state"] == State.END:
        #    self.end_pattern = True
        return

    def setup_scene(self):
        for i in range(1, 16):
            self.sequence_scenes.append(
                {
                    "background_cont": ["multiple_choice_container", ""],
                    "background": [
                        "img_bkg",
                        self.url
                        + str(i)
                        + "_Sfondo.png",
                    ],
                    "text": "Today it is a sunny day",
                    "choice_1": [
                        "img_1",
                        self.url
                        + str(i)
                        + "_Comp.png",
                    ],
                    "choice_2": [
                        "img_2",
                        self.url
                        + str(i)
                        + "_Distr.png",
                    ],
                    "choice_3": [
                        "img_3",
                        self.url
                        + str(i)
                        + "_Target.png",
                    ],
                }
            )
        self.sequence_scenes.append(
            {
                "background_cont": ["multiple_choice_container", ""],
                "background": [
                    "img_bkg",
                    self.url+ "17_Sfondo.png",
                ],
                "text": "Almost ended",
            }
        )
        self.sequence_scenes.append(
            {
                "background_cont": ["multiple_choice_container", ""],
                "background": [
                    "img_bkg",
                    self.url+ "18_Sfondo.png",
                ],
                "text": "The end",
            }
        )
        return

if __name__ == "__main__":
        pattern_name = rospy.get_param("/pattern_name/")
        test = rospy.get_param("/test_" + pattern_name + "/")
        test_input = rospy.get_param("/test_input_" + pattern_name + "/")
        test_id = rospy.get_param("/test_id_" + pattern_name + "/")
        url = rospy.get_param("/url_" + pattern_name + "/")
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pattern")
        pattern_script_path = pck_path + f"/pattern_scripting/{pattern_name}.json"
        with open(pattern_script_path, "r") as read_file:
            script = json.load(read_file)
        try:
            rospy.init_node(pattern_name)
            bc = MultipleChoiceDecisionManager(pattern_name, script, test_id, pattern_script_path, url)
            service_server = HarmoniServiceServer(name=pattern_name+"_decision", service_manager=bc)
            rospy.loginfo(f"START from the first step of {pattern_name} pattern.")
            if test:
                bc.start()
            else:
                service_server.update_feedback()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
