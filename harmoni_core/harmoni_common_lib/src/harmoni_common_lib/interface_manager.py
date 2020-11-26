#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import warnings


class HarmoniInterfaceManager(object):
    """
    The HarmoniInterface class implements a template class for interfacing between
    a service server and the actual implementation of the logic and functionality
    of a given node, also known as a Harmoni_Unit.

    Children of the service should overwrite the functionality of the parent functions
    with their own logic and functionality. Not all children should overwrite all functions
    as it wouldn't make sense for a microphone node to 'do' anything. 

    """

    def __init__(self):
        """ The service setup will instantiate the publishers, subscribers,
        class variables and clients.
        """
        rospy.loginfo(f"Initializing the interface manager")
        # Default variables

    def open(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return
    
    def next(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def pause(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return
    def resume(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def previous(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def terminate(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def play(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def started(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def completed(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return

    def finished(self, message):
        """ Make a request of another service, such as a web service

        Raises:
            NotImplementedError: To be used, this function should be overwritten by the child class.
        """
        raise NotImplementedError()
        return



