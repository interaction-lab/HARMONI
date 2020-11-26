#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import warnings
import websocket
import yaml
import _thread as thread
import time
import json

class HarmoniWebsocketClient(object):
    """
    The Harmoni Websocket client class implements a template class for interfacing between
    an external server and the client. 
    Class to send and receive messages to a websocket server.
    """

    def __init__(self, ip, port="", secure=False, openmessage="Hello"):
        """ The client setup will start the socket connection.
        Args:
            ip (str): url of websocket server
            port (int): port of server
            secure (bool): if the server is https (true) or not (false)
        """
        self.message = json.dumps(openmessage)
        #websocket.enableTrace(True)
        if port=="":
            
            if secure:
                endpoint = 'wss://' + ip
            else:
                endpoint= 'ws://' + ip
        else:
            
            if secure:
                endpoint = 'wss://' + ip + ':' + str(port)
            else:
                endpoint = 'ws://' + ip + ':' + str(port)
        rospy.loginfo("Connecting to websocket: "+endpoint)
        self.ws = websocket.WebSocketApp(endpoint,
                                on_message = self.on_message,
                                on_error = self.on_error,
                                on_close = self.on_close)
        self.ws.on_open = self.on_open
        self.ws.run_forever()
        

    def on_message(self, message):
        rospy.loginfo("Receving the message {message}")
        return

    def on_error(self, error):
        rospy.loginfo(error)
        return

    def on_close(self):
        rospy.loginfo("### closed ###")
        return

    def on_open(self):
        def run(*args):
            self.ws.send(self.message)
        thread.start_new_thread(run, ())
        rospy.loginfo("### open ###")
        return

    def send(self, message):
        rospy.loginfo("Send the message {message}")
        self.ws.send(json.dumps(message))
        return
    

    

