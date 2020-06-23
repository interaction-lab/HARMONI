#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.constants import *
from collections import defaultdict


class DialogueState():
    SENSING = "pc_microphone_default"
    SPEECH_DETECTING = "harmoni_stt_default"
    DIALOGING = "harmoni_lex_default"
    SYNTHETIZING = "harmoni_tts_default"
    SPEAKING = "pc_speaker_default"
    EXPRESSING = "pc_face_default"
    MOVING = ""


class DialogingPattern(HarmoniServiceManager, object):
    """
    Dialoging pattern class
    """

    def __init__(self, sequence, loop):
        """Init the behavior pattern and setup the clients"""
        self.sequence = sequence
        self.loop = loop
        self.count = -1
        self.count_loop = -1
        self.end_sequence = False
        self.end_single_loop = False
        self.end_looping = False
        self.action_info = {
            DialogueState.DIALOGING: {"router": Router.DIALOGUE.value, "action_goal": ActionType.REQUEST},
            DialogueState.SENSING: {"router": Router.SENSOR.value, "action_goal": ActionType.ON},
            DialogueState.SPEAKING: {"router": Router.ACTUATOR.value, "action_goal": ActionType.REQUEST},
            DialogueState.SYNTHETIZING: {"router": Router.ACTUATOR.value, "action_goal": ActionType.REQUEST},
            DialogueState.EXPRESSING: {"router": Router.ACTUATOR.value, "action_goal": ActionType.REQUEST},
            DialogueState.MOVING: {"router": Router.ACTUATOR.value, "action_goal": ActionType.REQUEST},
            DialogueState.SPEECH_DETECTING: {"router": Router.DETECTOR.value, "action_goal": ActionType.ON}
        }
        self.state = State.INIT
        super().__init__(self.state)
        self.router_names = [enum.value for enum in list(Router)]
        rospy.loginfo(f"Dialogue Pattern needs these routers: {self.router_names}")
        self.router_clients = defaultdict(HarmoniActionClient)
        for rout in self.router_names:
            self.router_clients[rout] = HarmoniActionClient()
        rospy.loginfo("Clients created")
        for rout, client in self.router_clients.items():
            client.setup_client(rout, self._result_callback, self._feedback_callback)
        rospy.loginfo("Behavior interface action clients have been set up!")

    def _result_callback(self, result):
        """ Do something when result has been received """
        rospy.loginfo("The result has been received")
        data = result
        if result["do_action"]:
            if not self.end_sequence:
                rospy.loginfo(f"Results says to do sequence {data['message']}")
                self.do_sequence(data['message'])
            elif not self.end_looping and self.end_sequence:
                rospy.loginfo(f"Results says to do loop {data['message']}")
                self.do_loop(data)['message']
        else:
            # if the dialogue intent is finalized
            # END THE BEHAVIOR PATTERN
            print("END BEHAVIOR PATTERN. DO SOMETHING ELSE NOW.")
        return

    def _feedback_callback(self, feedback):
        """ Send the feedback state to the Behavior Pattern tree to decide what to do next """
        rospy.logdebug("The feedback recieved is %s and nothing more" % feedback)
        return

    def start(self, action_goal, child_server, router, optional_data):
        """Start the Behavior Pattern sending the first goal to the child"""
        self.state = State.START
        rate = ""
        super().start(rate)
        # try:
        self.state = State.REQUEST
        rospy.loginfo(f"Sending the goal: {action_goal} to the router {router}")
        self.router_clients[router].send_goal(action_goal=action_goal, optional_data=optional_data, child_server=child_server)
        self.state = State.SUCCESS
        # except:
        #    self.state = State.FAILED
        return

    def stop(self, router):
        """Stop the Behavior Pattern """
        super().stop()
        try:
            self.router_clients[router].cancel_goal()
            self.state = State.SUCCESS
        except Exception as E:
            self.state = State.FAILED
        return

    def pause(self):
        """Pause the Behavior Pattern """
        super().pause()
        return

    def update(self, state):
        super().update(state)
        return

    def _get_action_info(self, action):
        """Get action info """
        self.state = action
        child_server = action
        router = self.action_info[child_server]["router"]
        action_goal = self.action_info[child_server]["action_goal"]
        return (child_server, router, action_goal)

    def do_sequence(self, data):
        """
        Do sequence, Update state and send the goal according to the current state

        Args:
            data: are the optional data to input to the service
        """
        self.count += 1
        action = self.sequence[self.count]
        if isinstance(action, list):
            for item in action:  # If it is an array, it means that is a parallel actions, so I start multiple goals
                [child_server, router, action_goal] = self._get_action_info(item)
        else:
            [child_server, router, action_goal] = self._get_action_info(action)
        optional_data = data
        self.start(action_goal, child_server, router, optional_data)
        self.update(self.state)
        if self.count == len(self.sequence):
            print("End of the sequence")
            self.end_sequence = True
        rospy.loginfo(f"The count is {self.count}")
        return

    def do_loop(self, data):
        """
        Do loop, Update state and send the goal according to the current state

        Args:
            data: are the optional data to input to the service
        """
        # self.count += 1

        action = self.loop[self.count]
        if isinstance(action, list):
            for item in action:  # If it is an array, it means that is a parallel actions, so I start multiple goals
                [child_server, router, action_goal] = self._get_action_info(item)
        else:
            [child_server, router, action_goal] = self._get_action_info(action)
        optional_data = data
        self.update(self.state)
        self.start(self, action_goal, child_server, router, optional_data)
        if self.count == len(self.loop):
            print("End of the single loop")
            self.end_single_loop = True
            self.count_loop += 1
            self.count = -1
            if self.end_looping:
                print("End looping")
        return

    def stop(self):
        pass


def main():
    trigger_intent = "Hey"
    parallel = [DialogueState.SPEAKING, DialogueState.EXPRESSING]
    sequence = [DialogueState.DIALOGING, DialogueState.SYNTHETIZING, parallel]
    loop = [DialogueState.SENSING, DialogueState.SPEECH_DETECTING, DialogueState.DIALOGING, DialogueState.SYNTHETIZING, parallel]
    try:
        rospy.init_node("dialoging_pattern")
        dp = DialogingPattern(sequence, loop)
        dp.do_sequence(data=trigger_intent)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
