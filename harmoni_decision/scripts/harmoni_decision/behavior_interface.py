#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import numpy as np
from collections import defaultdict
from harmoni_common_lib.constants import ActionType
from harmoni_common_lib.action_client import HarmoniActionClient


class HarmoniBehaviorInterface():
    """
    The behavior interface can act as a Client or as a Subscriber according to the BehaviorPattern it should run
    Test the behavior interface client side.
    """

    def __init__(self, router_names, subscriber_names):
        """ Init behavior interface"""
        [array_router, array_subscriber] = self.get_array_names(router_names, subscriber_names)
        self.router_names = array_router
        self.subscriber_names = array_subscriber
        self.router_clients = defaultdict(HarmoniActionClient)
        self.setup_behavior_interface()

    def get_array_names(self, router_names, subscriber_names):
        array_router = []
        array_subscriber = []
        for name in router_names:
            array_router.append(name)
        for name in subscriber_names:
            array_subscriber.append(name)
        return(array_router, array_subscriber)

    def setup_behavior_interface(self):
        """Setup behavior clients and subscribers """
        for rout in self.router_names:
            self.router_clients[rout] = HarmoniActionClient()
        for rout, client in self.router_clients.items():
            client.setup_client(rout, self.execute_result_callback, self.execute_feedback_callback)
        rospy.loginfo("Behavior interface action clients have been set up")
        # Implement a setup_subscriber as well
        return

    def execute_result_callback(self, result):
        """ Do something when result has been received """
        rospy.loginfo("The result has been received")
        return

    def execute_feedback_callback(self, feedback):
        """ Send the feedback state to the Behavior Pattern tree to decide what to do next """
        rospy.logdebug("The feedback is %s" %feedback)
        return

    def send_goal(self, action_goal, child_server, router, optional_data):
        self.router_clients[router].send_goal(action_goal=action_goal, optional_data=optional_data, child_server=child_server)
        return

    
def test(service, hi, wav_file, tts_input, dialogue_input, face_input, display_input):
    if service == "microphone":
        rospy.loginfo("Send the goal listening to the SensorRouter")
        hi.send_goal(action_goal=ActionType.ON, child_server=service, router="sensor", optional_data="")
    elif service == "lex":
        rospy.loginfo("Send the goal dialoging to the DialogueRouter")
        hi.send_goal(action_goal=ActionType.REQUEST, child_server=service, router="dialogue", optional_data=dialogue_input)
    elif service == "speaker":
        file_handle = wav_file
        data = np.fromfile(file_handle, np.uint8)[24:] #Loading wav file
        data = data.astype(np.uint8).tostring()
        rospy.loginfo("Send the goal speaking to the ActuatorRouter")
        hi.send_goal(action_goal=ActionType.REQUEST, child_server=service, router="actuator", optional_data=str(data))
    elif service == "tts":
        rospy.loginfo("Send the goal synthetizing to the ActuatorRouter")
        hi.send_goal(action_goal=ActionType.REQUEST, child_server=service, router="actuator", optional_data=tts_input)
    elif service == "face":
        rospy.loginfo("Send the goal expressing to the ActuatorRouter")
        hi.send_goal(action_goal=ActionType.REQUEST, child_server=service, router="actuator", optional_data=face_input)
    elif service == "web":
        rospy.loginfo("Send the goal expressing to the ActuatorRouter")
        hi.send_goal(action_goal=ActionType.REQUEST, child_server="web", router="actuator", optional_data=display_input)
    return

def main():
    try: 
        interface_name = "behavior_interface"
        rospy.init_node(interface_name + "_node")
        router_names = rospy.get_param("/routers/")
        subscriber_names = rospy.get_param("/subscribers/")
        rospy.loginfo("Set up the %s" %interface_name)
        test_service = rospy.get_param("/test_service/")
        wav_file = rospy.get_param("/wav_file/")
        tts_input = rospy.get_param("/tts_input_text/")
        dialogue_input = rospy.get_param("/dialogue_input_text/")
        face_input = rospy.get_param("/face_input/")
        display_input = rospy.get_param("/display_input/")
        hi = HarmoniBehaviorInterface(router_names, subscriber_names)
        if test_service != "":
            rospy.loginfo("The service to be tested is %s" %test_service)
            test(test_service, hi, wav_file, tts_input, dialogue_input, face_input, display_input)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
