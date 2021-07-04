#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import rosparam
from collections import deque, defaultdict
from harmoni_common_lib.action_client import HarmoniActionClient
import warnings


class HarmoniServiceManager(object):
    """
    The HarmoniService class implements a template class for interfacing between
    a service server and the actual implementation of the logic and functionality
    of a given node, also known as a Harmoni_Unit.

    Children of the service should overwrite the functionality of the parent functions
    with their own logic and functionality. Not all children should overwrite all functions
    as it wouldn't make sense for a microphone node to 'do' anything.

    """

    def __init__(self, name):
        """The service setup will instantiate the publishers, subscribers,
        class variables and clients.

        Args:
            name (str): Name of the service (useful for logging)
        """
        rospy.loginfo(f"Initializing the {name} service")
        rospy.set_param("robot_ip", "192.168.100.172") # TODO check where's the best position to set this parameter
        # Default variables
        self.name = name
        self.service_clients = defaultdict(HarmoniActionClient)
        self.configured_services = []  # available services
        self.client_results = defaultdict(deque)  # store state of the service

    def test(self):
        """Tests the setup has successfully completed and the unit is ready to
        be used
        """
        rospy.loginfo(f"Sucessfully reached {self.name}")
        return True

    def request(self, rate):
        """Make a request of another service, such as a web service

        When request returns it should return the result of the request and set status

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def do(self, data):
        """Will start a atomic action. This could be executing a movement or speech
        For long actions use do()

        When do returns the status should be set to succeeded or failed

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def start(self):
        """Will start a long running action. This could be reading hardware and publishing
        it to a topic or running a behavior pattern. For atomic actions use do()

        Start actions can be long running or indefinate.

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def pause(self):
        """Will interupt the long running action, preventing it from continuing until start
        is called again.

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.logwarn("service pause() not implemented. NOTE: This is currently required for goal preemption to work on longrunning processes.")
        return

    def stop(self):
        """Will interupt the long running action

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def reset_init(self):
        """Resets variables such that if start is called will start from beginning

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        rospy.logwarn("Class reset not implemented, add a reset to your node")
        return
