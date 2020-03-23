#!/usr/bin/env python

from std_msgs.msg import String, Bool
from cordial_manager.msg import *
import time
import json
import os
import sys
import actionlib
import rospy
import roslib


DECISION_TIME_OUT =  rospy.Duration.from_sec(6000)

class DecisionState():
    SUCCESS = 0
    FAILURE = 1
    EMERGENCY = 2

class DecisionManager():
    """Commands a series of interactions to the interaction manager
    
        Actions commanding: do_interaction - ManagerAction
        Actions receiving: none
    """
    def __init__(self):
        #Initialize the state variable
        self.state = DecisionState.SUCCESS
        self.index = 0
        self.failure_counter = 0
        self.action_feedback = {}
        self.action_result = {}
        story_first_name = rospy.search_param('first_story')
        story_first = rospy.get_param(story_first_name)
        story_sec_name = rospy.search_param('second_story')
        story_second = rospy.get_param(story_sec_name)
        story_third_name = rospy.search_param('third_story')
        story_third = rospy.get_param(story_third_name)
        print(story_first)
        # Setup clients for all of the different interaction blocks
        topic_name = "do_interaction"
        self.action_client = actionlib.SimpleActionClient(topic_name, CordialAction)

        rospy.loginfo("Waiting for the server")
        self.action_client.wait_for_server()
        rospy.loginfo("Server is ready")

        # TODO Initialize with list of interactions and interaction failure options
        self.success_interaction_name = ['greeting1',str(story_first),'wait',str(story_second), 'wait', str(story_third), 'lastwait','goodbye']
        self.failure_interaction_name = ['fail1', 'fail2','wait','fail2', 'wait', 'fail2', 'lastwait', 'goodbye']
        #self.success_interaction_name = [ 'greeting1','goodbye']
        #self.failure_interaction_name = ['fail1', 'goodbye']

        # Set response to action to default - will be changed by callback
        all_interactions = self.success_interaction_name + self.failure_interaction_name
        for interaction in all_interactions:
            self.action_result[interaction] = {
                    "do_continue": False,
                    "message": ""}
            self.action_feedback[interaction] = {
                    "state": ""}
        
        # Send out the first action
        self.send_interaction_request(self.success_interaction_name[self.index])
        return
       

    def send_interaction_request(self, interaction_name, optional_data=""):
        """Send goal to interaction manager """
        # Create goal
        goal = CordialGoal()
        goal.action = interaction_name
        goal.optional_data = optional_data
        rospy.loginfo("The goal to be sent is:"+ str(goal))

        # Send goal
        self.action_client.send_goal(goal,
                                    done_cb=self.interaction_done_callback,
                                    feedback_cb=self.interaction_feedback_callback)
        #print("Waiting for the result")
        #self.action_client.wait_for_result(DECISION_TIME_OUT)
        return


    def interaction_done_callback(self, terminal_state, result):
        """Handle completed interaction"""
        # Read message
        rospy.loginfo("Heard back from: "+ result.action + " terminal state is: "+  str(terminal_state) +" and the result is: "+ str(result))
        self.action_result[result.action]["do_continue"] = result.do_continue
        self.action_result[result.action]["message"] = result.message 

        # Continue if successful
        if self.action_result[result.action]["do_continue"]:
            self.failure_counter = 0
            self.index += 1
            self.state = DecisionState.SUCCESS
            if self.index < len(self.success_interaction_name): # not done with all interactions
                self.send_interaction_request(self.success_interaction_name[self.index])
        else:
            print("Do continue False")
            if self.action_result[result.action]["message"] == 'understanding':
                print("Failed understanding")
                self.failure_counter += 1
                self.state = DecisionState.FAILURE
                if self.failure_counter < 4: # 3 re-try allowed
                    self.send_interaction_request(self.failure_interaction_name[self.index])
                else:
                    self.send_interaction_request(self.success_interaction_name[self.index])
            elif self.action_result[result.action]["message"] == 'emergency':
                self.state = DecisionState.EMERGENCY
        return


    def interaction_feedback_callback(self, feedback):
        self.action_feedback[feedback.action]["state"] = feedback.states
        return


if __name__ == '__main__':
    rospy.init_node("decision_controller_node", anonymous=True)
    DecisionManager()
    rospy.spin()