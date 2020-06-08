#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.behavior_pattern import BehaviorPatternService
from harmoni_common_lib.constants import *

class DialogueState():
    SENSING = "pc_microphone_default"
    DIALOGING = "harmoni_lex_default"
    SYNTHETIZING = "harmoni_tts_default"
    SPEAKING = "pc_speaker_default"
    EXPRESSING = "pc_face_default"
    #MOVING = ""
    
class DialogingPattern(BehaviorPatternService):
    """
    Dialoging pattern class
    """
    def __init__(self):
        """Init the behavior pattern """
        self.parallel = [DialogueState.SPEAKING, DialogueState.EXPRESSING]
        self.sequence = [DialogueState.DIALOGING, DialogueState.SYNTHETIZING, self.parallel]
        self.loop = self.sequence.insert(0, DialogueState.SENSING)
        self.action_info = {
            DialogueState.DIALOGING: {"router": Router.DIALOGUE, "action_goal": ActionType.REQUEST},
            DialogueState.SENSING: {"router": Router.SENSOR, "action_goal": ActionType.ON} ,
            DialogueState.SYNTHETIZING: {"router": Router.ACTUATOR, "action_goal": ActionType.REQUEST} ,
            DialogueState.SPEAKING: {"router": Router.ACTUATOR, "action_goal": ActionType.REQUEST} ,
            DialogueState.EXPRESSING: {"router": Router.ACTUATOR, "action_goal": ActionType.REQUEST} ,
            DialogueState.MOVING: {"router": Router.ACTUATOR, "action_goal": ActionType.REQUEST}
        }

    def get_action_info(self):
        """Get action info """
        self.action = self.sequence[self.count]
        self.state = self.action
        child_server = self.action
        router = self.action_info[child_server]["router"]
        action_goal = self.action_info[child_server]["action_goal"]
        return (child_server, router, action_goal)


    def start(self, data):
        """Start dialoging """
        self.count = 0
        [child_server, router, action_goal] = self.get_action_info()
        optional_data = data
        super().start(self, action_goal, child_server, router, optional_data)
        return

    def update(self, data):
        """Update state and send the goal according to the current state"""
        self.count += 1
        [child_server, router, action_goal] = self.get_action_info()
        optional_data = data
        self.update(self.state)
        super().start(self, action_goal, child_server, router, optional_data)
        return

    def stop(self):
        pass
        

def main():
    pass


if __name__ == "__main__":
    main()
