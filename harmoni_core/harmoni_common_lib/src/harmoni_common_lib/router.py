#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import os
import yaml
from harmoni_common_lib.constants import State
from harmoni_common_lib.action_server import HarmoniActionServer
from harmoni_common_lib.action_client import HarmoniActionClient
from collections import defaultdict

TIMEOUT_FOR_RESULT = 600
TIMEOUT_FOR_SERVER = 10


class HarmoniRouter(HarmoniActionServer, object):
    """
    A control provider receives some request from the manager and send the
    correspondings request action to the child.
    This class provides basic router functionality which the subclasses of
    router can exploit
    """

    def __init__(self, router_name, child_constants_names, last_event):
        """ Initialization of the variables """
        rospy.loginfo(f"Initializing HarmoniRouter: {router_name}")
        self.timeout_for_result = TIMEOUT_FOR_RESULT
        self.timeout_for_server = TIMEOUT_FOR_SERVER
        self.last_event = last_event
        self.children_clients = defaultdict(HarmoniActionClient)
        child_names = self._get_child_name(child_constants_names)
        rospy.loginfo("The %s children are %s" % (router_name, child_names))
        for child in child_names:
            self.children_clients[child] = HarmoniActionClient()
        self.router_name = router_name
        self.result_received = False

    def setup_actions(
        self, execute_goal_result_callback, execute_goal_feedback_callback
    ):
        """ Setup clients of each subclass and the server of the router"""
        for child, client in self.children_clients.items():
            client.setup_client(
                child, execute_goal_result_callback, execute_goal_feedback_callback
            )
            rospy.loginfo(f"{self.router_name} Router: Set up client for " + child)
        super().__init__(self.router_name, self.execute_goal_received_callback)
        return

    def setup_conditional_startup(self, condition_event, checked_event):
        """ Set condition for starting the action """
        while condition_event != checked_event:
            rospy.loginfo(f"{self.router_name} Router: Waiting for event to finish")
            rospy.Rate(1)
        rospy.loginfo(
            f"{self.router_name} Router: Conditional event ended successfully"
        )
        return

    def execute_goal_received_callback(self, goal):
        """
        Receiving the request (server role)
        Check if setting up a conditional startup or not
        Sending the goal request to the server (client role)
        """
        if len(goal.optional_data) < 500:
            rospy.loginfo(
                f"{self.router_name} Router-Message: \n action_type type: {goal.action_type} \n optional_data: {goal.optional_data} \n child: {goal.resource}"
            )
        else:
            rospy.loginfo(
                f"{self.router_name} Router-Message: \n action_type type: {goal.action_type} \n optional_data: (too large to print) \n child: {goal.resource}"
            )
        # TODO This hack is going to need some work
        wait = True
        if (
            goal.resource
            == "pc_face_default"
            # or goal.resource == "pc_speaker_default"
        ):
            wait = False
        # rospy.loginfo("The request data are:" + str(goal))
        # if goal.condition != "uncondition":  # check if the action is conditioned by another event or not
        # self.setup_conditional_startup(goal.condition, self.last_event)
        rospy.loginfo(
            f"{self.router_name} Router: Start a goal request to the {goal.resource}"
        )
        self.children_clients[goal.resource].send_goal(
            action_goal=goal.action_type,
            resource=goal.resource,
            optional_data=goal.optional_data,
            condition="",
            time_out=self.timeout_for_result,
            wait=wait,
        )
        rospy.loginfo(f"{self.router_name} Router: message sent")
        if not wait:
            rospy.loginfo(
                f"{self.router_name} Router: Not waiting, submitting an early return"
            )
            self.send_result(True, "Early return")
            self.result_received = True
        return

    def _get_child_name(self, child_constants_names):
        """Get children from config file"""
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("HARMONI/")
        with open(
            path[0] + "HARMONI/harmoni_core/harmoni_decision/config/configuration.yaml"
        ) as file:
            repos = yaml.load(file, Loader=yaml.FullLoader)
        child_names = []
        for repo in repos:
            for child in child_constants_names:
                if child in repos[repo]:
                    for i in range(len(repos[repo][child])):
                        child_names.append(
                            repo + "_" + child + "_" + repos[repo][child][i]
                        )
        return child_names
