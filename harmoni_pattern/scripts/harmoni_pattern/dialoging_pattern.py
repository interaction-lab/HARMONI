#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.behavior_pattern import BehaviorPatternService
from harmoni_common_lib.constants import *

class DialogueState():
    SENSING = "pc_microphone_default"
    SPEECH_DETECTING = "harmoni_stt_default"
    DIALOGING = "harmoni_lex_default"
    SYNTHETIZING = "harmoni_tts_default"
    SPEAKING = "pc_speaker_default"
    EXPRESSING = "pc_face_default"
    MOVING = ""
    
class DialogingPattern(BehaviorPatternService):
    """
    Dialoging pattern class
    """
    def __init__(self):
        """Init the behavior pattern """
        self.action_info = {
            DialogueState.DIALOGING: {"router": Router.DIALOGUE, "action_goal": ActionType.REQUEST},
            DialogueState.SENSING: {"router": Router.SENSOR, "action_goal": ActionType.ON} ,
            DialogueState.SYNTHETIZING: {"router": Router.ACTUATOR, "action_goal": ActionType.REQUEST} ,
            DialogueState.SPEAKING: {"router": Router.ACTUATOR, "action_goal": ActionType.REQUEST} ,
            DialogueState.EXPRESSING: {"router": Router.ACTUATOR, "action_goal": ActionType.REQUEST} ,
            DialogueState.MOVING: {"router": Router.ACTUATOR, "action_goal": ActionType.REQUEST},
            DialogueState.SPEECH_DETECTING: {"router": Router.DETECTOR, "action_goal": ActionType.ON}
        }

    def _get_action_info(self, action):
        """Get action info """
        self.state = self.action
        child_server = self.action
        router = self.action_info[child_server]["router"]
        action_goal = self.action_info[child_server]["action_goal"]
        return (child_server, router, action_goal)

    def initialise_pattern(self, data, array):
        """
        Init state and send the goal according to the initial state
        
        Args:
            data: are the optional data to input to the service
            array: it is the array of the sequence of steps to accomplish the pattern, e.g., self.parallel, self.loop    
        """
        self.count = 0
        action = array[self.count]
        if isinstance(action, list):
            for item in action: # If it is an array, it means that is a parallel actions, so I start multiple goals
                [child_server, router, action_goal] = self._get_action_info(item)
        else:
            [child_server, router, action_goal] = self._get_action_info(action)
        optional_data = data
        super().start(self, action_goal, child_server, router, optional_data)
        return

    def update_goal(self, data, array):
        """
        Update state and send the goal according to the current state
        
        Args:
            data: are the optional data to input to the service
            array: it is the array of the sequence of steps to accomplish the pattern, e.g., self.parallel, self.loop    
        """
        self.count += 1
        action = array[self.count]
        if isinstance(action, list):
            for item in action: # If it is an array, it means that is a parallel actions, so I start multiple goals
                [child_server, router, action_goal] = self._get_action_info(item)
        else:
            [child_server, router, action_goal] = self._get_action_info(action)
        optional_data = data
        self.update(self.state)
        super().start(self, action_goal, child_server, router, optional_data)
        return

    def stop(self):
        pass
        

def main():
    parallel = [DialogueState.SPEAKING, DialogueState.EXPRESSING]
    sequence = [DialogueState.DIALOGING, DialogueState.SYNTHETIZING, self.parallel]
    loop = [DialogueState.SENSING, DialogueState.SPEECH_DETECTING, DialogueState.DIALOGING, DialogueState.SYNTHETIZING, self.parallel]
    try:
        dp = DialogingPattern()
        dp.initialise_pattern(sequence)
    except:
        pass


if __name__ == "__main__":
    main()
