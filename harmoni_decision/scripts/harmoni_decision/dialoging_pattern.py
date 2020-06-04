#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.behavior_pattern import BehaviorPatternService
from harmoni_common_lib.constants import *


class DialogingPattern(BehaviorPatternService):
    """
    Dialoging pattern class
    """
    def __init__(self):
        """Init the behavior pattern """
        self.start(action_goal = ActionType.REQUEST, child_server="harmoni_lex_def", router="dialogue", optional_data="Hey")
        

def main():
    pass


if __name__ == "__main__":
    main()
