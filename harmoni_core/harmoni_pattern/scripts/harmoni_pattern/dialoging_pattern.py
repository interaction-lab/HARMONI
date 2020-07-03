#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.constants import *
from harmoni_common_lib.helper_functions import HelperFunctions
from harmoni_common_lib.constants import State, RouterDetector
from collections import defaultdict
from collections import deque
from time import time


class DialogueState:
    LISTENING = "pc_microphone_default"
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

    def __init__(self, name, sequence, loop, sensors, detectors):
        """Init the behavior pattern and setup the clients"""
        self.service_names = []
        self.name = name
        self.sequence = sequence
        self.loop = loop

        self.count = self.count_loop = -1
        self.end_sequence = self.end_single_loop = self.end_looping = False
        self.dialogue_state = ""
        self.action_info = {
            DialogueState.DIALOGING: {
                "action_goal": ActionType.REQUEST,
                "resource": "",
            },
            DialogueState.LISTENING: {"action_goal": ActionType.ON, "resource": ""},
            # DialogueState.NOTLISTENING: {"action_goal": ActionType.PAUSE, "resource": ""},
            DialogueState.SPEAKING: {"action_goal": ActionType.REQUEST, "resource": ""},
            DialogueState.SYNTHETIZING: {
                "action_goal": ActionType.REQUEST,
                "resource": "",
            },
            DialogueState.EXPRESSING: {
                "action_goal": ActionType.REQUEST,
                "resource": "",
            },
            DialogueState.MOVING: {"action_goal": ActionType.REQUEST, "resource": ""},
            DialogueState.SPEECH_DETECTING: {
                "action_goal": ActionType.ON,
                "resource": "",
            },
        }

        self.state = State.INIT
        super().__init__(self.state)

        list_repos = HelperFunctions.get_all_repos()
        for repo in list_repos:
            [repo_child_list, child_list] = HelperFunctions.get_service_list_of_repo(
                repo
            )
            for child in child_list:
                self.service_names.extend(HelperFunctions.get_child_list(child))
        rospy.loginfo(f"Dialogue Pattern needs these services: {self.service_names}")
        self.service_clients = defaultdict(HarmoniActionClient)

        for client in self.service_names:
            self.service_clients[client] = HarmoniActionClient()
        rospy.loginfo("Clients created")

        for cl, client in self.service_clients.items():
            client.setup_client(cl, self._result_callback, self._feedback_callback)
        rospy.loginfo("Behavior interface action clients have been set up!")

        # Sensors and detectors should be launched on startup
        # detectors will have callbacks which store a queue of results
        # In the loop the last item will be read and returned as the result
        for sensor in sensors:
            (
                action_goal,
                resource,
                service,
                service_name,
                _is_detector,
                _is_sensor,
            ) = self._get_action_info(sensor)
            self.service_clients[service].send_goal(
                action_goal=action_goal,
                optional_data="",
                resource=resource,
                wait=False,
            )

        self.detector_queue = defaultdict(deque)
        for detector in detectors:
            (
                action_goal,
                resource,
                service,
                service_name,
                _is_detector,
                _is_sensor,
            ) = self._get_action_info(detector)

            self.service_clients[service].send_goal(
                action_goal=action_goal,
                optional_data="",
                resource=resource,
                wait=False,
            )
            rospy.loginfo(
                f"The detector service is {service} !@#$!@$(*&#!%)(!*#&$!()_@#$&"
            )
            service_id = HelperFunctions.get_child_id(service)
            rospy.Subscriber(
                RouterDetector.stt.value + "default",
                String,
                self._detecting_callback,
                callback_args=service_name,
                queue_size=1,
            )
            self.detector_queue[service_name] = deque()
        return

    def _result_callback(self, result):
        """ Do something when result has been received """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(f"The result callback message was {len(result['message'])} long")
        if result["do_action"]:
            # If you are not done working through your sequence, do the next step
            if not self.end_sequence:
                self.do_sequence(result["message"])
            # If you are at the end of your sequence, work through the loop
            elif not self.end_looping and self.end_sequence:
                self.do_loop(result["message"])
        else:
            # If the result did not say to do anything next
            print("Callback specified stop.")
        return

    def _feedback_callback(self, feedback):
        """ Send the feedback state to the Behavior Pattern tree to decide what to do next """
        rospy.logdebug("The feedback recieved is %s and nothing more" % feedback)
        return

    def _detecting_callback(self, data, service_name):
        """Callback function from subscribing to the detector topic """
        # HERE WE SHOULD GET THE DATA FOR PASSING THEM TO THE NEXT STEP
        data = data.data
        rospy.loginfo(f"Heard back from {service_name} detector: {data}")
        self.detector_queue[service_name].append({"time": time(), "data": data})
        return

    def request_step(self, action, optional_data, wait=True):
        """Send goal request to appropriate child"""
        if isinstance(action, list):
            # If it is an array, it means that is a parallel actions, so I start multiple goals
            rospy.loginfo("Running action in parallel-ish (launching multiple goals)")
            for i, sub_action in enumerate(action, start=1):
                # wait on the last item in a list TODO: update to wait on all items after all are sent
                # handle multiple goals with recursion
                self.request_step(sub_action, optional_data, wait=(i == len(action)))
        else:
            # Here is where the magic starts
            (
                action_goal,
                resource,
                service,
                service_name,
                _is_detector,
                _is_sensor,
            ) = self._get_action_info(action)

            self.state = State.REQUEST

            # HANDLE SENSOR AND DETECTOR CASE
            if _is_sensor:
                rospy.logwarn("Sensor should be set up during init")
                wait = False
                self._result_callback({"do_action": True, "message": ""})
            elif _is_detector:  # if detector, get last message from topic
                rospy.loginfo("getting result from the detector")
                call_time = time()
                result = {"time": 0, "data": "_not_data"}
                if len(self.detector_queue[service_name]) < 0:
                    result = self.detector_queue[service_name].popleft()
                r = rospy.Rate(1)

                while result["time"] < call_time:
                    rospy.loginfo(f"got old message {result['data']}")
                    if len(self.detector_queue[service_name]) > 0:
                        result = self.detector_queue[service_name].popleft()
                    r.sleep()

                rospy.loginfo(f"Detected message {result['data']}")
                self._result_callback({"do_action": True, "message": result["data"]})
                return
            else:

                # Log details about the service:
                rospy.loginfo(f"Sending the following to the {service_name} service")
                if len(optional_data) < 500:
                    rospy.loginfo(
                        f"Message: \n action_goal type: {action_goal} \n optional_data: {optional_data} \n child: {resource}"
                    )
                else:
                    rospy.loginfo(
                        f"Message: \n action_goal type: {action_goal} \n optional_data: (too large to print) \n child: {resource}"
                    )

                self.service_clients[service].send_goal(
                    action_goal=action_goal,
                    optional_data=optional_data,
                    resource=resource,
                    wait=wait,
                )
                rospy.loginfo("Goal sent.")
                self.state = State.SUCCESS
            # except:
            #    self.state = State.FAILED
        self.update(self.state)
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
        """Helper function to get the server, resource, and goal associated with an action"""
        rospy.loginfo("The action is %s" % action)
        self.dialogue_state = action
        service = action
        print(self.action_info[service])
        resource = self.action_info[service]["resource"]
        action_goal = self.action_info[service]["action_goal"]
        service_name = HelperFunctions.get_service_name(service)
        _is_detector = HelperFunctions.check_if_detector(service_name)
        _is_sensor = HelperFunctions.check_if_sensor(service_name)
        return action_goal, resource, service, service_name, _is_detector, _is_sensor

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
        rospy.loginfo(f"************* Starting SEQUENCE step: {self.count}")
        action = self.sequence[self.count]
        rospy.loginfo(f"Sequence step action is {action}!!!!!!!!!!!!!!!!!!!")
        self.request_step(action, data)
        rospy.loginfo(f"************ End of SEQUENCE step: {self.count} *************")
        return

    def do_loop(self, data):
        """
        Do loop, Update state and send the goal according to the current state

        Args:
            data: are the optional data to input to the service
        """
        self.count_loop += 1
        self.count_loop = self.count_loop % len(self.loop)
        rospy.loginfo(f"************* Starting LOOP step: {self.count_loop}")
        action = self.loop[self.count_loop]
        rospy.loginfo(f"Loop step action is {action}!!!!!!!!!!!!!!!!!!!")
        """
        if isinstance(action, list):
            # If it is an array, it means that is a parallel actions, so I start multiple goals
            rospy.loginfo("Running action in parallel!")
            i = 0
            for item in action:
                i += 1
                [resource, service, action_goal] = self._get_action_info(item)

                self.request_step(
                    action_goal, resource, service, data, wait=(i == len(action))
                )
        else:
            [resource, service, action_goal] = self._get_action_info(action)
            rospy.loginfo("Request step")
            self.request_step(action_goal, resource, service, optional_data)
        self.update(self.state)
        """
        self.request_step(action, data)

        if self.count_loop == len(self.loop):
            print("******** End of the single loop")
            self.end_single_loop = True
            self.count_loop += 1
            self.count = -1
            if self.end_looping:
                print("********** End looping")
        rospy.loginfo(f"************ End of LOOP step: {self.count_loop} *************")
        return

    def stop(self):
        pass


def main():
    pattern_name = "dialoging"
    trigger_intent = "Hey"
    parallel = [
        DialogueState.EXPRESSING,
        DialogueState.SPEAKING,
    ]
    loop = [
        # DialogueState.LISTENING,
        DialogueState.SPEECH_DETECTING,
        DialogueState.DIALOGING,
        DialogueState.SYNTHETIZING,
        parallel,
    ]
    sequence = [DialogueState.DIALOGING, DialogueState.SYNTHETIZING, parallel]
    sensors = [DialogueState.LISTENING]
    detectors = [DialogueState.SPEECH_DETECTING]

    try:
        rospy.init_node(pattern_name)
        # Initialize the pattern with pattern sequence/loop
        dp = DialogingPattern(pattern_name, sequence, loop, sensors, detectors)
        rospy.loginfo("Set up. Starting first step of dialogue pattern.")
        dp.start(data=trigger_intent)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
