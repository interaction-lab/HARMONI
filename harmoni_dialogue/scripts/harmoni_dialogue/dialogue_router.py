#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.router import HarmoniRouter
from harmoni_common_lib.constants import RouterDialogue, Router

class HarmoniDialogueRouter(HarmoniRouter):
    """
    The dialogue router aims to handle the dialogue, interfacing with external or internal services
    """

    def __init__(self, router_name, last_event):
        """ Init router"""
        child_constants_names = [enum.name for enum in list(RouterDialogue)]
        super(HarmoniDialogueRouter, self).__init__(router_name, child_constants_names, last_event)

    def setup_router(self):
        self.setup_actions(self.execute_result_callback, self.execute_feedback_callback)
        rospy.loginfo("Dialogue router actions have been set up")
        return

    def execute_result_callback(self, result):
        """ Do something when result has been received """
        rospy.loginfo("The result received is %s" %result)
        self.send_result(result["do_action"], result["message"])
        return

    def execute_feedback_callback(self, feedback):
        """ Send the feedback backward when feedback has been received """
        rospy.logdebug("The feedback received is %s" %feedback)
        self.send_feedback(feedback["state"])
        return


def main():
    try: 
        router_name = Router.DIALOGUE.value
        rospy.init_node(router_name)
        last_event = ""  
        hr = HarmoniDialogueRouter(router_name, last_event)
        hr.setup_router()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
