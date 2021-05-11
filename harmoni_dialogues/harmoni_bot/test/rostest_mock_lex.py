#!/usr/bin/env python3


PKG = 'test_harmoni_bot'
# Common Imports
import unittest, rospy, roslib, sys
#from unittest.mock import Mock, patch
# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
import os, io
from nose.tools import assert_true
import requests
from threading import Thread
from http.server import BaseHTTPRequestHandler, HTTPServer

#The response from lex is: {'ResponseMetadata': {'RequestId': 'ff1edd6b-b4b1-4858-ba39-95b1e471c041', 'HTTPStatusCode': 200, 'HTTPHeaders': {'x-amzn-requestid': 'ff1edd6b-b4b1-4858-ba39-95b1e471c041', 'x-amz-lex-nlu-intent-confidence': 'eyJzY29yZSI6MS4wfQ==', 'x-amz-lex-message': 'My name is QT. Nice to meet you!', 'x-amz-lex-active-contexts': 'bnVsbA==', 'x-amz-lex-dialog-state': 'Fulfilled', 'x-amz-lex-bot-version': '1', 'x-amz-lex-slots': 'e30=', 'x-amz-lex-session-id': '2020-12-10T11:28:22.029Z-cIlssWrn', 'x-amz-lex-alternative-intents': 'W3siaW50ZW50TmFtZSI6IkFNQVpPTi5GYWxsYmFja0ludGVudCIsIm5sdUludGVudENvbmZpZGVuY2UiOm51bGwsInNsb3RzIjp7fX1d', 'x-amz-lex-intent-name': 'Welcome', 'x-amz-lex-message-format': 'PlainText', 'date': 'Thu, 10 Dec 2020 11:28:26 GMT', 'content-type': 'text/plain; charset=utf-8', 'content-length': '0'}, 'RetryAttempts': 0}, 'contentType': 'text/plain; charset=utf-8', 'intentName': 'Welcome', 'nluIntentConfidence': {'score': 1.0}, 'alternativeIntents': [{'intentName': 'AMAZON.FallbackIntent', 'nluIntentConfidence': None, 'slots': {}}], 'slots': {}, 'message': 'My name is QT. Nice to meet you!', 'messageFormat': 'PlainText', 'dialogState': 'Fulfilled', 'botVersion': '1', 'sessionId': '2020-12-10T11:28:22.029Z-cIlssWrn', 'audioStream': <botocore.response.StreamingBody object at 0x7f2e61ed62e0>}


class MockLex(BaseHTTPRequestHandler):

    def do_GET(self):
            # Process an HTTP GET request and return a response with an HTTP 200 status.
            rospy.loginfo("Http GET")
            self.send_response(requests.codes.ok, message=None)
            # Add response headers.
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.end_headers()
            return



class TestLex(unittest.TestCase):

        

    def setUp(self):
        rospy.init_node("test_lex", log_level=rospy.INFO)
        self.text = rospy.get_param("test_lex_input")
        self.text = "Hello"
        self.result = False
        # NOTE currently no feedback, status, or result is received.
        # rospy.Subscriber("/bot_default/feedback", harmoniFeedback, self.feedback_cb)
        # rospy.Subscriber("/bot_default/status", GoalStatus, self.status_cb)
        # rospy.Subscriber("/bot_default/result", harmoniResult, self.result_cb)
        # Configure mock server.
        rospy.loginfo("TestLex: Started up. waiting for lex startup")
        self.mock_server_port = 3210
        self.mock_server = HTTPServer(('localhost', self.mock_server_port), MockLex)
        # Start running mock server in a separate thread.
        # Daemon threads automatically shut down when the main process exits.
        self.mock_server_thread = Thread(target=self.mock_server.serve_forever)
        self.mock_server_thread.setDaemon(True)
        self.mock_server_thread.start()
        rospy.loginfo("TestLex: Started")
        
    

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
    rospy.loginfo("test_lex started")
    rospy.loginfo("TestLex: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, 'test_lex', TestLex, sys.argv)
    

if __name__ == "__main__":
    main()
