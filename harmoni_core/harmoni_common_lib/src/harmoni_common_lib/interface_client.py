#!/usr/bin/env python3

# Importing the libraries
import roslaunch
import rospy
import json
import ast
from harmoni_common_lib.websocket_client import HarmoniWebsocketClient
from std_msgs.msg import String
import harmoni_common_lib.helper_functions as hf


class HarmoniClientInterface(HarmoniWebsocketClient, object):
    def __init__(self, ip, port="", secure=False, message="Hello", client_manager):
        self.ip = ip
        self.port = port
        self.secure = secure
        self.message = message
        self.patient_id = ""
        self.client_manager = client_manager
        #rospy.Subscriber("/harmoni/actuating/web/default/listen_click_event", String, self.start, queue_size=1)
        #self.web_pub = rospy.Publisher('/harmoni/actuating/web/default/set_view', String, queue_size=1)
        rospy.loginfo("Initializing client. Wait for opening socket.")


    def open_socket(self, data):
        # init websocket connection
        if(data!=""):
            self.message=data
            super().__init__(self.ip, self.port, self.secure, self.message)
        

    def on_message(self, message):
        rospy.loginfo("The message received is: " +message)
        self._handle_message(message)
        return
    
    def send_message(self,request, message):
        switcher = {
            "STARTED": self.client_manager.started(message),
            "COMPLETED": self.client_manager.completed(message),
            "FINISHED": self.client_manager.finished(message)
        }
        # Get the function from switcher dictionary
        func = switcher.get(request, lambda: "Invalid request")
        # Execute the function
        print(func)
        return

    def _handle_message(self, message):
        message=ast.literal_eval(message)
        rospy.loginfo("The request received is " + message["action"])
        request = message["action"]
        switcher = {
            "OPEN": self.client_manager.open(message),
            "NEXT": self.client_manager.next(message),
            "PAUSE": self.client_manager.pause(message),
            "RESUME": self.client_manager.resume(message),
            "PREVIOUS": self.client_manager.previous(message),
            "TERMINATE": self.client_manager.terminate(message),
            "PLAY": self.client_manager.play(message)
        }
        # Get the function from switcher dictionary
        func = switcher.get(request, lambda: "Invalid request")
        # Execute the function
        func
        return