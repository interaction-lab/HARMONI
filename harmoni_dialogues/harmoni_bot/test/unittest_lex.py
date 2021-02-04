#!/usr/bin/env python3


PKG = 'test_harmoni_bot'
# Common Imports
import unittest, rospy, roslib, sys
#from unittest.mock import Mock, patch
# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.constants import State
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from std_msgs.msg import String
import os, io
import ast
from harmoni_bot.aws_lex_service import AWSLexService
import json
#The response from lex is: {'ResponseMetadata': {'RequestId': 'ff1edd6b-b4b1-4858-ba39-95b1e471c041', 'HTTPStatusCode': 200, 'HTTPHeaders': {'x-amzn-requestid': 'ff1edd6b-b4b1-4858-ba39-95b1e471c041', 'x-amz-lex-nlu-intent-confidence': 'eyJzY29yZSI6MS4wfQ==', 'x-amz-lex-message': 'My name is QT. Nice to meet you!', 'x-amz-lex-active-contexts': 'bnVsbA==', 'x-amz-lex-dialog-state': 'Fulfilled', 'x-amz-lex-bot-version': '1', 'x-amz-lex-slots': 'e30=', 'x-amz-lex-session-id': '2020-12-10T11:28:22.029Z-cIlssWrn', 'x-amz-lex-alternative-intents': 'W3siaW50ZW50TmFtZSI6IkFNQVpPTi5GYWxsYmFja0ludGVudCIsIm5sdUludGVudENvbmZpZGVuY2UiOm51bGwsInNsb3RzIjp7fX1d', 'x-amz-lex-intent-name': 'Welcome', 'x-amz-lex-message-format': 'PlainText', 'date': 'Thu, 10 Dec 2020 11:28:26 GMT', 'content-type': 'text/plain; charset=utf-8', 'content-length': '0'}, 'RetryAttempts': 0}, 'contentType': 'text/plain; charset=utf-8', 'intentName': 'Welcome', 'nluIntentConfidence': {'score': 1.0}, 'alternativeIntents': [{'intentName': 'AMAZON.FallbackIntent', 'nluIntentConfidence': None, 'slots': {}}], 'slots': {}, 'message': 'My name is QT. Nice to meet you!', 'messageFormat': 'PlainText', 'dialogState': 'Fulfilled', 'botVersion': '1', 'sessionId': '2020-12-10T11:28:22.029Z-cIlssWrn', 'audioStream': <botocore.response.StreamingBody object at 0x7f2e61ed62e0>}


class TestLex(unittest.TestCase):

    def __init__(self, *args):
        super(TestLex, self).__init__(*args)

    def setUp(self):
        self.text = "Hello"
        self.result = False
        rospy.loginfo("TestLex: Started up. waiting for lex startup")
        self.aws_service = AWSLexService("test_aws", param={"user_id":"chris_testing",  "bot_name": "GreeterPrototype","bot_alias": "test_bot", "region_name": "us-west-2"})
        rospy.loginfo("TestLex: Started")
    
    
    def test_request_response(self):
        # Send a request to the real API server and store the response.
        response = self.aws_service.request(self.text)
        # Confirm that the request-response cycle completed successfully.
        rospy.loginfo(response)
        if response["response"]==State.SUCCESS:
            rospy.loginfo("The response succeed")
            self.result = True #set the response to true if the request succeeded
        assert(self.result == True)

def main():
    #TODO convert to a test suite so that setup doesn't have to run over and over.
    import rosunit
    rospy.loginfo("test_lex started")
    rospy.loginfo("TestLex: sys.argv: %s" % str(sys.argv))
    rosunit.unitrun(PKG, 'test_lex', TestLex, sys.argv)

if __name__ == "__main__":
    main()
