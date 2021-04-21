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
import json
import requests
from threading import Thread
from http.server import BaseHTTPRequestHandler, HTTPServer

class MockPolly(BaseHTTPRequestHandler):

    def do_GET(self):
            # Process an HTTP GET request and return a response with an HTTP 200 status.
            rospy.loginfo("Http GET")
            response = {'ResponseMetadata': {'RequestId': 'ff1edd6b-b4b1-4858-ba39-95b1e471c041', 'HTTPStatusCode': 200, 'HTTPHeaders': {'x-amzn-requestid': 'ff1edd6b-b4b1-4858-ba39-95b1e471c041', 'x-amz-lex-nlu-intent-confidence': 'eyJzY29yZSI6MS4wfQ==', 'x-amz-lex-message': 'My name is QT. Nice to meet you!', 'x-amz-lex-active-contexts': 'bnVsbA==', 'x-amz-lex-dialog-state': 'Fulfilled', 'x-amz-lex-bot-version': '1', 'x-amz-lex-slots': 'e30=', 'x-amz-lex-session-id': '2020-12-10T11:28:22.029Z-cIlssWrn', 'x-amz-lex-alternative-intents': 'W3siaW50ZW50TmFtZSI6IkFNQVpPTi5GYWxsYmFja0ludGVudCIsIm5sdUludGVudENvbmZpZGVuY2UiOm51bGwsInNsb3RzIjp7fX1d', 'x-amz-lex-intent-name': 'Welcome', 'x-amz-lex-message-format': 'PlainText', 'date': 'Thu, 10 Dec 2020 11:28:26 GMT', 'content-type': 'text/plain; charset=utf-8', 'content-length': '0'}, 'RetryAttempts': 0}, 'contentType': 'text/plain; charset=utf-8', 'intentName': 'Welcome', 'nluIntentConfidence': {'score': 1.0}, 'alternativeIntents': [{'intentName': 'AMAZON.FallbackIntent', 'nluIntentConfidence': None, 'slots': {}}], 'slots': {}, 'message': 'My name is QT. Nice to meet you!', 'messageFormat': 'PlainText', 'dialogState': 'Fulfilled', 'botVersion': '1', 'sessionId': '2020-12-10T11:28:22.029Z-cIlssWrn', 'audioStream': ""}
            # Add response status code.
            self.send_response(requests.codes.ok, message=response)

            # Add response headers.
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.end_headers()
            # Add response content.
            response_content = json.dumps(response)
            self.wfile.write(response_content.encode('utf-8'))
            #self.send_response(json.dumps(response))
            # Add response headers.
            return

    def do_POST(self):
            # Process an HTTP POST request and return a response with an HTTP 200 status.
            rospy.loginfo("Http POST")
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            post_data = post_data.decode("utf-8")
            if str(post_data) == "Hello":
                response = {'ResponseMetadata': {'RequestId': 'ff1edd6b-b4b1-4858-ba39-95b1e471c041', 'HTTPStatusCode': 200, 'HTTPHeaders': {'x-amzn-requestid': 'ff1edd6b-b4b1-4858-ba39-95b1e471c041', 'x-amz-lex-nlu-intent-confidence': 'eyJzY29yZSI6MS4wfQ==', 'x-amz-lex-message': 'My name is QT. Nice to meet you!', 'x-amz-lex-active-contexts': 'bnVsbA==', 'x-amz-lex-dialog-state': 'Fulfilled', 'x-amz-lex-bot-version': '1', 'x-amz-lex-slots': 'e30=', 'x-amz-lex-session-id': '2020-12-10T11:28:22.029Z-cIlssWrn', 'x-amz-lex-alternative-intents': 'W3siaW50ZW50TmFtZSI6IkFNQVpPTi5GYWxsYmFja0ludGVudCIsIm5sdUludGVudENvbmZpZGVuY2UiOm51bGwsInNsb3RzIjp7fX1d', 'x-amz-lex-intent-name': 'Welcome', 'x-amz-lex-message-format': 'PlainText', 'date': 'Thu, 10 Dec 2020 11:28:26 GMT', 'content-type': 'text/plain; charset=utf-8', 'content-length': '0'}, 'RetryAttempts': 0}, 'contentType': 'text/plain; charset=utf-8', 'intentName': 'Welcome', 'nluIntentConfidence': {'score': 1.0}, 'alternativeIntents': [{'intentName': 'AMAZON.FallbackIntent', 'nluIntentConfidence': None, 'slots': {}}], 'slots': {}, 'message': 'My name is QT. Nice to meet you!', 'messageFormat': 'PlainText', 'dialogState': 'Fulfilled', 'botVersion': '1', 'sessionId': '2020-12-10T11:28:22.029Z-cIlssWrn', 'audioStream': ""}
                # Add response status code.
                response_content = json.dumps(response["message"])
                self.send_response(requests.codes.ok, message=response_content.encode('utf-8'))
                #response_content = json.dumps(response["message"])
                #self.wfile.write(response_content.encode('utf-8'))
            else:
                self.send_error(400)
            # Add response headers.
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.end_headers()
            return



class TestPolly(unittest.TestCase):

    def __init__(self, *args):
        super(TestPolly, self).__init__(*args)
        

    def setUp(self):
        self.text = "Hello"
        self.result = False
        rospy.loginfo("TestPolly: Started up. waiting for polly startup")
        self.mock_server_port = 3210
        self.mock_server = HTTPServer(('localhost', self.mock_server_port), MockPolly)
        # Start running mock server in a separate thread.
        # Daemon threads automatically shut down when the main process exits.
        self.mock_server_thread = Thread(target=self.mock_server.serve_forever)
        self.mock_server_thread.setDaemon(True)
        self.mock_server_thread.start()
        rospy.loginfo("TestPolly: Started")
        
    def test_request_response(self):
        url = 'http://localhost:{port}/users'.format(port=self.mock_server_port)
        rospy.loginfo(f"The url is {url}")
        # Send a request to the mock API server and store the response.
        response = requests.post(url, data=self.text)
        if response:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_polly started")
    rospy.loginfo("TestPolly: sys.argv: %s" % str(sys.argv))
    rosunit.unitrun(PKG, 'test_polly', TestPolly, sys.argv)

if __name__ == "__main__":
    main()
