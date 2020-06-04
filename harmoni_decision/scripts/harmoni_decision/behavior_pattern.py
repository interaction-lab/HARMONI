#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib import constants


# The main pattern will act as a service
# subscriptions do not need to be dynamic
class BehaviorPatternService(HarmoniServiceManager):
    """Abstract class defining common variables/functions for behavior patterns

    Args:
        controller(BehaviorController): Class instance where all data and commands pass through.
    """

    def ___init__(self):
        raise NotImplementedError

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def pause(self):
        raise NotImplementedError

# The main function will be structured like chilren,
# with a service manager that recieves instructions and controls
# the pattern


def main():
    pass


if __name__ == "__main__":
    main()
