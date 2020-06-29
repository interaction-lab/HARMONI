#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.constants import *
from collections import defaultdict


class DialogueState:
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
            DialogueState.DIALOGING: {
                "router": Router.DIALOGUE.value,
                "action_goal": ActionType.REQUEST,
            },
            DialogueState.SENSING: {
                "router": Router.SENSOR.value,
                "action_goal": ActionType.ON,
            },
            DialogueState.SPEAKING: {
                "router": Router.ACTUATOR.value,
                "action_goal": ActionType.REQUEST,
            },
            DialogueState.SYNTHETIZING: {
                "router": Router.ACTUATOR.value,
                "action_goal": ActionType.REQUEST,
            },
            DialogueState.EXPRESSING: {
                "router": Router.ACTUATOR.value,
                "action_goal": ActionType.REQUEST,
            },
            DialogueState.MOVING: {
                "router": Router.ACTUATOR.value,
                "action_goal": ActionType.REQUEST,
            },
            DialogueState.SPEECH_DETECTING: {
                "router": Router.DETECTOR.value,
                "action_goal": ActionType.ON,
            },
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
        rospy.loginfo("The result of the request has been received")
        data = result
        rospy.loginfo(f"The result callback message was {len(data['message'])} long")
        if result["do_action"]:
            # If you are not done working through your sequence, do the next step
            if not self.end_sequence:
                self.do_sequence(data["message"])
            # If you are at the end of your sequence, work through the loop
            elif not self.end_looping and self.end_sequence:
                self.do_loop(data["message"])
        else:
            # If the result did not say to do anything next
            print("Callback specified stop.")
        return

    def _feedback_callback(self, feedback):
        """ Send the feedback state to the Behavior Pattern tree to decide what to do next """
        rospy.logdebug("The feedback recieved is %s and nothing more" % feedback)
        return

    def request_step(self, action_goal, child_server, router, optional_data, wait=True):
        """Send goal request to appropriate child"""
        # try:
        self.state = State.REQUEST
        rospy.loginfo(f"Sending the following to the {router} router")
        if len(optional_data) < 500:
            rospy.loginfo(
                f"Message: \n action_goal type: {action_goal} \n optional_data: {optional_data} \n child: {child_server}"
            )
        else:
            rospy.loginfo(
                f"Message: \n action_goal type: {action_goal} \n optional_data: (too large to print) \n child: {child_server}"
            )

        self.router_clients[router].send_goal(
            action_goal=action_goal,
            optional_data=optional_data,
            child_server=child_server,
            wait=wait,
        )
        rospy.loginfo("Goal sent.")
        self.state = State.SUCCESS
        # except:
        #    self.state = State.FAILED
        return

    def start(self, data):
        """Send goal request to appropriate child"""
        self.state = State.START
        rate = ""
        super().start(rate)
        # try:
        self.do_sequence(data=data)
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
        """Helper function to get the server, router, and goal associated with an action"""
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
        if self.count == len(self.sequence):
            print("DONE WITH THE WHOLE SEQUENCE")
            self.end_sequence = True
            return
        rospy.loginfo(f"************* Starting sequence step: {self.count}")
        action = self.sequence[self.count]
        if isinstance(action, list):
            # If it is an array, it means that is a parallel actions, so I start multiple goals
            rospy.loginfo("Running action in parallel!")
            i = 0
            for item in action:
                i += 1
                [child_server, router, action_goal] = self._get_action_info(item)
                self.request_step(
                    action_goal, child_server, router, data, wait=(i == len(action))
                )
        else:
            [child_server, router, action_goal] = self._get_action_info(action)
            self.request_step(action_goal, child_server, router, data)
        # self.update(self.state)

        rospy.loginfo(f"************ End of sequence step: {self.count} *************")
        return

    def do_loop(self, data):
        """
        Do loop, Update state and send the goal according to the current state

        Args:
            data: are the optional data to input to the service
        """
        self.count_loop += 1
        rospy.loginfo(f"Starting loop step: {self.count_loop}")

        action = self.loop[self.count_loop]
        rospy.loginfo(f"Loop step action is {action}")
        if isinstance(action, list):
            for (
                item
            ) in (
                action
            ):  # If it is an array, it means that is a parallel actions, so I start multiple goals
                [child_server, router, action_goal] = self._get_action_info(item)
                self.request_step(
                    self, action_goal, child_server, router, optional_data
                )
        else:
            [child_server, router, action_goal] = self._get_action_info(action)
            self.request_step(self, action_goal, child_server, router, optional_data)
        optional_data = data
        # self.update(self.state)

        if self.count_loop == len(self.loop):
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
    parallel = [DialogueState.EXPRESSING, DialogueState.SPEAKING]
    # parallel = [DialogueState.EXPRESSING]
    sequence = [
        DialogueState.DIALOGING,
        DialogueState.SYNTHETIZING,
        parallel,
        DialogueState.DIALOGING,
        DialogueState.SYNTHETIZING,
        parallel,
        DialogueState.DIALOGING,
        DialogueState.SYNTHETIZING,
        parallel,
    ]
    loop = [
        DialogueState.SENSING,
        DialogueState.SPEECH_DETECTING,
        DialogueState.DIALOGING,
        DialogueState.SYNTHETIZING,
        parallel,
    ]
    try:
        rospy.init_node("dialoging_pattern")
        # Initialize the pattern with pattern sequence/loop
        dp = DialogingPattern(sequence, loop)
        rospy.loginfo("Set up. Starting first step of dialogue pattern.")
        dp.start(data=trigger_intent)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
