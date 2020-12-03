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
import ast

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
        rospy.loginfo(f"Receiving the message {message}")
        message=ast.literal_eval(message)
        rospy.loginfo("The request received is " + message["action"])
        request = message["action"]
        rospy.loginfo("Request is "+request)
        if request=="OPEN":
            self.open(message)
        elif request=="PLAY":
            self.play_game(message)
        elif request=="NEXT":
            self.next(message)
        elif request=="REPEAT":
            self.repeat(message)
        elif request=="PAUSE":
            self.pause(message)
        elif request=="RESUME":
            self.resume(message)
        elif request=="PREVIOUS":
            self.previous(message)
        elif request=="TERMINATE":
            self.terminate(message)
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
        rospy.loginfo(f"Send the message {message}")
        self.ws.send(json.dumps(message))
        return
        

    def open(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("open")
        return

    def repeat(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("repeat")
        return

    
    def next(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("next")
        return

    def play_game(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("open")
        return

    def pause(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("open")
        return
    def resume(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("open")
        return

    def previous(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("previous")
        return

    def terminate(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.loginfo("open")
        return

    

