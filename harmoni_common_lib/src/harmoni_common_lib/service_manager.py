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
        rospy.loginfo("Init the service manager status")
        self.status = status

    def update(self, status):
        self.status = status
        rospy.loginfo("Update the status to %i" %status)
        return

    def test(self):
        """ Test the hardware, sending default action """
        return

    def start(self, rate):
        """ Start reading or processing data """
        return

    def pause(self):
        """ Pause reading or processing data """
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

    def __init__(self, status):
        rospy.loginfo("Init the direct service manager")
        self.response_received = False
        self.actuation_completed = False
        self.result_msg = ""
        self.status = status

    def test(self):
        """ Test the hardware, sending default action """
        return

    def update(self, status, actuation_completed="", response_received="", result_msg=""):
        self.response_received = response_received  # True if action completed
        self.status = status  # Used IF logic can dictate control flow
        self.result_msg = result_msg  # String
        self.actuation_completed = actuation_completed
        return

    def request(self, rate):
        """ Do a request """
        return

    def do(self, data):
        """ Do an action """
        return data

    def reset_init(self):
        """ Reset harware variables to initial state """
        self.response_received = False
        self.result_msg = ""
        return
