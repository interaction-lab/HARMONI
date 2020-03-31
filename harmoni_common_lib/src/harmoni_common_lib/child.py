#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
from action_server import ActionServer


class HardwareControl():

    def __init__(self, input_data):
        # Initialize variables
        self.input_data = input_data
        # Initialize server
        self.server = ActionServer()

    def setup(self, child):
        self.server.setup_server(child)
        return


class ExternalService():

    def __init__(self, input_data):
        # Initialize variables
        self.input_data = input_data
        # Initialize server
        self.server = ActionServer()

    def setup(self, child):
        self.server.setup_server(child)
        return


class InternalService():

    def __init__(self, data_type_sub, data_type_pub, topic_sub, topic_pub):
        # Initialization of subscriber and publisher info
        self.data_type_sub = data_type_sub
        self.topic_sub = topic_sub
        self.data_type_pub = data_type_pub
        self.topic_pub = topic_pub
        # Initialize server
        self.server = ActionServer()

    def setup(self, child):
        self.server.setup_server(child)
        return


class HarwareReading():

    def __init__(self, data_type, topic):
        # Initialization of publisher info
        self.data_type = data_type
        self.topic = topic
        # Initialize server
        self.server = ActionServer()

    def setup(self, child):
        self.server.setup_server(child)
        return
