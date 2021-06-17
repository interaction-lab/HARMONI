#!/usr/bin/env python3


PKG = "test_harmoni_face"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from std_msgs.msg import String
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType
from collections import deque
import os, io
import ast


class TestFace(unittest.TestCase):

    def setUp(self):
        """
        Set up the client for requesting to harmoni_face
        """
        rospy.init_node("test_face", log_level=rospy.INFO)
        # self.data = rospy.get_param(
        #     "test_face_input"
        # ) 
        self.face_data = str([
            {'start': 0, 'time': 2,  'type': 'au', 'id': 'au13', 'pose': 1},
            {'start': 1, 'time': 2,  'type': 'action', 'id': 'breath_face'},
            {'start': 2, 'time': 2,  'type': 'action', 'id': 'saucy_face'},
            {'start': 0.075,'time': 2, 'type': 'viseme', 'id': 'POSTALVEOLAR'}
        ])

        self.eye_data = str([
            {'start': 0, 'time': 2,  'type': 'gaze', 'id':'target', 'point': [1, 5, 10]},
            {'start': 2, 'time': 2,  'type': 'gaze', 'id':'target', 'point': [10, 5, 1]},
            {'start': 4, 'time': 2,  'type': 'gaze', 'id':'target', 'point': [5, 10, 1]},
        ])


        self.instance_id = rospy.get_param("instance_id")
        self.result_eyes = False
        self.result_mouth = False
        self.name_mouth = ActuatorNameSpace.face.name + "_mouth_" + self.instance_id
        self.service_client_mouth = HarmoniActionClient(self.name_mouth)
        self.client_result_mouth = deque()
        self.service_client_mouth.setup_client(self.name_mouth, self.result_mouth_cb, self.feedback_mouth_cb)
        self.name_eyes = ActuatorNameSpace.face.name + "_eyes_" + self.instance_id
        self.service_client_eyes = HarmoniActionClient(self.name_eyes)
        self.client_result_eyes = deque()
        self.service_client_eyes.setup_client(self.name_eyes, self.result_eyes_cb, self.feedback_eyes_cb)
        # NOTE currently no feedback, status, or result is received.
        rospy.Subscriber(
            "/harmoni_face_mouth_default/feedback", harmoniFeedback, self.feedback_mouth_cb
        )
        rospy.Subscriber("/harmoni_face_mouth_default/status", GoalStatus, self.status_mouth_cb)
        rospy.Subscriber(
            "/harmoni_face_mouth_default/result", harmoniResult, self.result_mouth_cb
        )
        rospy.Subscriber(
            "/harmoni_face_eyes_default/feedback", harmoniFeedback, self.feedback_eyes_cb
        )
        rospy.Subscriber("/harmoni_face_eyes_default/status", GoalStatus, self.status_eyes_cb)
        rospy.Subscriber(
            "/harmoni_face_eyes_default/result", harmoniResult, self.result_eyes_cb
        )
        rospy.loginfo("TestFace: Started up. waiting for face startup")
        rospy.loginfo("TestFace: Started")

    def feedback_mouth_cb(self, data):
        rospy.loginfo(f"Feedback: {data}")
        self.result_mouth = False

    def status_mouth_cb(self, data):
        rospy.loginfo(f"Status: {data}")
        self.result_mouth = False

    def result_mouth_cb(self, data):
        rospy.loginfo(f"Result: {data}")
        self.result_mouth = True

    def feedback_eyes_cb(self, data):
        rospy.loginfo(f"Feedback: {data}")
        self.result_eyes = False

    def status_eyes_cb(self, data):
        rospy.loginfo(f"Status: {data}")
        self.result_eyes = False

    def result_eyes_cb(self, data):
        rospy.loginfo(f"Result: {data}")
        self.result_eyes = True
    
    def test_request_response_mouth(self):
        print("################******************* MOUTH TEST STARTING")
        rospy.loginfo(f"The input data is {self.face_data}")
        self.service_client_mouth.send_goal(
            action_goal=ActionType.DO.value,
            optional_data=self.face_data,
            wait=True,
        )
        print("################******************* MOUTH TEST Ending")
        assert self.result_mouth == True
    
    def test_request_response_eyes(self):
        print("*******************@@@@@@@@@@@@@@@@ Eye test starting")
        rospy.loginfo(f"The input data is {self.eye_data}")
        self.service_client_eyes.send_goal(
            action_goal=ActionType.DO.value,
            optional_data=self.eye_data,
            wait=True,
        )
        print("*******************@@@@@@@@@@@@@@@@ Eye test ending")
        assert self.result_eyes == True
        input("Continue?")

def main():
    import rostest
    rospy.loginfo("test_face started")
    rospy.loginfo("TestFace: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_face", TestFace, sys.argv)


if __name__ == "__main__":
    main()
