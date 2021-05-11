#!/usr/bin/env python3


PKG = 'test_harmoni_tts'
# Common Imports
import unittest, rospy, roslib, sys
#from unittest.mock import Mock, patch
# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
import os, io
import requests
from threading import Thread
from http.server import BaseHTTPRequestHandler, HTTPServer

class MockPolly(BaseHTTPRequestHandler):

    def do_GET(self):
            # Process an HTTP GET request and return a response with an HTTP 200 status.
            rospy.loginfo("Http GET")
            self.send_response(requests.codes.ok, message=None)
            # Add response headers.
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.end_headers()
            return



class TestPolly(unittest.TestCase):
        
    def setUp(self):
        rospy.init_node("test_polly", log_level=rospy.INFO)
        self.text = rospy.get_param("test_polly_input")
        self.text = "Hello"
        self.result = False
        # NOTE currently no feedback, status, or result is received.
        # rospy.Subscriber("/tts_default/feedback", harmoniFeedback, self.feedback_cb)
        # rospy.Subscriber("/tts_default/status", GoalStatus, self.status_cb)
        # rospy.Subscriber("/tts_default/result", harmoniResult, self.result_cb)
        # Configure mock server.
        rospy.loginfo("TestPolly: Started up. waiting for polly startup")
        self.mock_server_port = 3210
        self.mock_server = HTTPServer(('localhost', self.mock_server_port), MockPolly)
        # Start running mock server in a separate thread.
        # Daemon threads automatically shut down when the main process exits.
        self.mock_server_thread = Thread(target=self.mock_server.serve_forever)
        self.mock_server_thread.setDaemon(True)
        self.mock_server_thread.start()
        rospy.loginfo("TestPolly: Started")
        
    

    def feedback_cb(self, data):
        print(f"Feedback: {data}")
        self.result = True

    def status_cb(self, data):
        print(f"Status: {data}")
        self.result = True
    
    def result_cb(self, data):
        print(f"Result: {data}")
        self.result = True
    
    def test_request_response(self):
        url = 'http://localhost:{port}/users'.format(port=self.mock_server_port)
        rospy.loginfo(f"The url is {url}")
        # Send a request to the mock API server and store the response.
        response = requests.get(url)
        self.result = True
        # Confirm that the request-response cycle completed successfully.
        print(response)
        assert(self.result == True)
        

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rostest
    rospy.loginfo("test_polly started")
    rospy.loginfo("TestPolly: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, 'test_polly', TestPolly, sys.argv)
    

if __name__ == "__main__":
    main()
