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
    A control provider receives some request from the manager and send the correspondings
    request action to the child.
    This class provides basic router functionality which the subclasses of router can exploit
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

    def setup_actions(self, execute_goal_result_callback, execute_goal_feedback_callback):
        """ Setup clients of each subclass and the server of the router"""
        for child, client in self.children_clients.items():
            client.setup_client(child, execute_goal_result_callback, execute_goal_feedback_callback)
            rospy.loginfo("Set up client for " + child)
        self.setup_server(self.router_name, self.execute_goal_received_callback)
        return

    def setup_conditional_startup(self, condition_event, checked_event):
        """ Set condition for starting the action """
        while condition_event != checked_event:
            rospy.loginfo("Waiting for event to finish")
            rospy.Rate(1)
        rospy.loginfo("Conditional event ended successfully")
        return

    def execute_goal_received_callback(self, goal):
        """ 
        Receiving the request (server role)
        Check if setting up a conditional startup or not
        Sending the goal request to the server (client role)
        """
        rospy.loginfo("The request data are:" + str(goal))
        # if goal.condition != "uncondition":  # check if the action is conditioned by another event or not
        #self.setup_conditional_startup(goal.condition, self.last_event)
        rospy.loginfo("Start a goal request to the child")
        self.children_clients[goal.child_server].send_goal(action_goal=goal.action_type, child_server=goal.child_server, optional_data=goal.optional_data, condition="", time_out=self.timeout_for_result)
        return

    def _get_child_name(self, child_constants_names):
        """Get children from config file"""
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("HARMONI/")
        with open(path[0] + 'HARMONI/harmoni_decision/config/configuration.yaml') as file:
            repos = yaml.load(file, Loader=yaml.FullLoader)
        child_names = []
        for repo in repos:
            for child in child_constants_names:
                if child in repos[repo]:
                    for i in range(len(repos[repo][child])):
                        child_names.append(repo + "_" + child + "_" + repos[repo][child][i])
        return child_names
