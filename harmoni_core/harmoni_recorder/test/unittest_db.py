#!/usr/bin/env python3


PKG = 'test_harmoni_recorder'
# Common Imports
import unittest, rospy, rospkg, roslib, sys
#from unittest.mock import Mock, patch
# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from harmoni_common_lib.constants import State
from std_msgs.msg import String
import os, io
import ast
from harmoni_recorder.mongodb_client import MongoDBClient


class TestDB(unittest.TestCase):
    def __init__(self, *args):
        super(TestDB, self).__init__(*args)

    def setUp(self):
        """
        Set up the client for requesting to harmoni_recorder
        """
        rospy.init_node("test_recorder", log_level=rospy.INFO)
        self.data = rospy.get_param(
            "test_db_input"
        ) 
        self.data = ast.literal_eval(self.data)
        self.mdb_client = MongoDBClient(client_uri="mongodb://172.18.3.3:27017/", username="root", password="example")
        self.instance_id = rospy.get_param("instance_id")
        self.result = False
        rospy.loginfo("TestDB: Started")

    def test_request_db(self):
        rospy.loginfo(f"The input data is {self.data}")
        client = self.mdb_client.get_client()
        id_inserted = client.harmoni.users.insert_one(self.data)
        if id_inserted:
            self.result=True
        assert self.result == True

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_recorder started")
    rospy.loginfo("TestDB: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_recorder", TestDB, sys.argv)


if __name__ == "__main__":
    main()
