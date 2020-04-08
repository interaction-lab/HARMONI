#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib


class HarmoniServiceManager(object):
    """
    Service manager for the harware reading and internal service servers.
    Individual service managers overwrite the parent public functions.
    """

    def __init__(self, status):
        rospy.loginfo("Init the service manager")
        self.status = status

    def test(self):
        """ Test the hardware, sending default action """
        rospy.loginfo("Test placeholder")
        return

    def start(self, rate):
        """ Start reading or processing data """
        return

    def stop(self):
        """ Stop reading or processing data """
        return

    def reset_init(self):
        """ Reset harware variables to initial state """
        return


class HarmoniExternalServiceManager(object):
    """
    Service manager for the harware control and external service servers.
    Individual service managers overwrite the parent public functions.
    """

    def __init__(self, response_received, cont, result_msg):
        rospy.loginfo("Init the direct service manager")
        self.response_received = response_received  # True if action completed
        self.cont = cont  # Used IF logic can dictate control flow
        self.result_msg = result_msg  # String

    def test(self):
        """ Test the hardware, sending default action """
        return

    def request(self, rate):
        """ Do a request """
        return

    def do(self):
        """ Do an action """
        return

    def reset_init(self):
        """ Reset harware variables to initial state """
        return
