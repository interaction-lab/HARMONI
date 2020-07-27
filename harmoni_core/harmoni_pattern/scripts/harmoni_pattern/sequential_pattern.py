#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
import rospkg
import json
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
import threading


class SequentialPattern(HarmoniServiceManager, object):
    """
    Dialoging pattern class
    """

    def __init__(self, name, script):
        """Init the behavior pattern and setup the clients"""
        self.name = name
        self.script = script
        self.end_pattern = False
        self.scripted_services = set()  # services used in this script
        self.configured_services = []  # available services
        self.service_clients = defaultdict(HarmoniActionClient)
        self.client_results = defaultdict(deque)  # store state of the service
        self.script_set_index = 0

        self.state = State.INIT
        super().__init__(self.state)

        self.scripted_services = self._get_services(script)
        self._setup_clients()

        if script[self.script_set_index]["set"] == "setup":
            self.setup(script[self.script_set_index]["steps"])
            self.script_set_index += 1

        return

    def _setup_clients(self):
        """
        Set up all clients that have been configured in the
        harmoni_decision module
        """
        list_repos = HelperFunctions.get_all_repos()
        for repo in list_repos:
            [repo_child_list, child_list] = HelperFunctions.get_service_list_of_repo(
                repo
            )
            for child in child_list:
                self.configured_services.extend(HelperFunctions.get_child_list(child))

        for service in self.scripted_services:
            assert (
                service in self.configured_services
            ), "Scripted service has not been configured"

        for client in self.scripted_services:
            self.service_clients[client] = HarmoniActionClient(client)
            self.client_results[client] = deque()
        rospy.loginfo("Clients created")
        rospy.loginfo(
            f"{self.name} Pattern needs these services: {self.scripted_services}"
        )

        for cl, client in self.service_clients.items():
            client.setup_client(cl, self._result_callback, self._feedback_callback)
        rospy.loginfo("Behavior interface action clients have been set up!")
        return

    def _get_services(self, script):
        service_names = set()
        for s in script:
            steps = s["steps"]
            for step in steps:
                if isinstance(step, list):
                    for parallel_step in step:
                        service_names.add(next(iter(parallel_step)))
                else:
                    service_names.add(next(iter(step)))
        return service_names

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        rospy.loginfo("The result of the request has been received")
        rospy.loginfo(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_results[result["service"]].append(
            {"time": time(), "data": result["message"]}
        )
        # TODO add handling of errors and continue=False
        return

    def _feedback_callback(self, feedback):
        """ Send the feedback state to the Behavior Pattern tree to decide what to do next """
        rospy.logdebug("The feedback recieved is %s and nothing more" % feedback)
        #Check if the state is end, stop the behavior pattern
        #if feedback["state"] == State.END:
        #    self.end_pattern = True
        return

    def _detecting_callback(self, data, service_name):
        """Callback function from subscribing to the detector topic """
        # HERE WE SHOULD GET THE DATA FOR PASSING THEM TO THE NEXT STEP
        data = data.data
        self.client_results[service_name].append({"time": time(), "data": data})
        return

    def start(self):
        """Send goal request to appropriate child"""
        self.state = State.START
        super().start(rate=1)
        r = rospy.Rate(1)
        while self.script_set_index < len(self.script) and not rospy.is_shutdown():
            if self.script[self.script_set_index]["set"] == "setup":
                self.setup(self.script[self.script_set_index]["steps"])

            elif self.script[self.script_set_index]["set"] == "sequence":
                self.count = -1
                self.do_sequence(self.script[self.script_set_index]["steps"])

            elif self.script[self.script_set_index]["set"] == "loop":
                self.count = -1
                self.do_sequence(
                    self.script[self.script_set_index]["steps"], looping=True
                )
            elif self.end_pattern:
                #for client in self.scripted_services:
                #    self.stop(client)
                break
            self.script_set_index += 1
            r.sleep()
        return

    def stop(self, service):
        """Stop the Behavior Pattern """
        super().stop()
        try:
            self.service_clients[client].cancel_goal()
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

    def setup(self, children):
        for child in children:
            service = next(iter(child))
            details = child[service]

            assert details["resource_type"] in [
                "sensor",
                "detector",
            ], "Can only set up sensors or detectors"

            self.service_clients[service].send_goal(
                action_goal=ActionType[details["action_goal"]].value,
                optional_data="",
                wait=details["wait_for"],
            )

            if details["resource_type"] == "detector":
                # service_id = HelperFunctions.get_child_id(service)
                service_list = service.split("_")
                service_id = "_".join(service_list[1:-1])
                topic = f"/harmoni/detecting/{service_id}/default"
                rospy.loginfo(f"subscribing to {topic}")
                rospy.Subscriber(
                    topic,
                    String,
                    self._detecting_callback,
                    callback_args=service,
                    queue_size=1,
                )

        return

    def do_sequence(self, sequence, looping=False):
        """
        Do sequence, Update state and send the goal according to the current state

        Args:
            data: are the optional data to input to the service
        """

        result = None
        for cnt, step in enumerate(sequence, start=1):
            if rospy.is_shutdown():
                return
            rospy.loginfo(f"------------- Starting sequence step: {cnt}-------------")
            if result:
                rospy.loginfo(f"with prior result length ({len(result)})")
            else:
                rospy.loginfo("no prior result")
            result = self.request_step(step, result)
            rospy.loginfo(f"************* End of sequence step: {cnt} *************")

        if looping:
            rospy.loginfo("Done with a loop!")
            # TODO check on loop condition before continuing
            if not rospy.is_shutdown():
                self.do_sequence(sequence, looping=True)

        return

    def request_step(self, step, optional_data=None):
        """Send goal request to appropriate child"""
        if isinstance(step, list):
            threads = []
            # If it is an array, it means that is a parallel actions, so I start multiple goals
            # In the current implementation parallel actions return values will not get passed on
            rospy.loginfo("Running action in parallel-ish (launching multiple goals)")
            for i, sub_action in enumerate(step, start=1):
                t = threading.Thread(
                    target=self.request_step, args=(sub_action, optional_data)
                )
                threads.append(t)
                t.start()
                # result = self.request_step(sub_action, optional_data)
            for t in threads:
                t.join()
            result = None
            return result
        else:
            service = next(iter(step))
            details = step[service]
            rospy.loginfo(f"Step is {service} with details {details}")
            # rospy.loginfo(f"and optional_data {optional_data}")
            assert details["resource_type"] in [
                "sensor",
                "detector",
                "actuator",
                "service",
            ], "must specify resource type of each step"

            self.state = State.REQUEST

            # HANDLE SENSOR AND DETECTOR CASE
            if details["resource_type"] == "sensor":
                rospy.logwarn("Sensor should be set up during init")
                result = None

            elif details["resource_type"] == "detector":
                rospy.loginfo(f"Retrieving data from detector: {service}")
                if details["wait_for"] == "new":
                    return_data = self.get_new_result(service)
                else:
                    rospy.logwarn("Not waiting for a detector may return last result")
                    if len(self.client_results[service]) > 0:
                        return_data = self.client_results[service].popleft()["data"]
                    return_data = None

            else:  # Should be a request and response from an actuator or service
                if "trigger" in details.keys():
                    optional_data = details["trigger"]
                elif not optional_data:
                    optional_data = ""
                rospy.loginfo(
                    f"Sending goal to {service} optional_data len {len(optional_data)}"
                )
                self.service_clients[service].send_goal(
                    action_goal=ActionType[details["action_goal"]].value,
                    optional_data=optional_data,
                    wait=False,
                )
                rospy.loginfo(f"Goal sent to {service}")
                self.state = State.SUCCESS
                if details["wait_for"] == "new":
                    return_data = self.get_new_result(service)
                else:
                    rospy.logwarn("Not waiting for a detector may return last result")
                    if len(self.client_results[service]) > 0:
                        return_data = self.client_results[service].popleft()["data"]
                    else:
                        return_data = None

        self.update(self.state)
        return return_data

    def get_new_result(self, service):
        rospy.loginfo("getting result from the service")
        rospy.loginfo(f"Queue size is {len(self.client_results[service])}")
        rospy.loginfo(f"Queue is {self.client_results[service]}")

        call_time = time()
        result = {"time": 0, "data": "_the_queue_is_empty"}

        if len(self.client_results[service]) > 0:
            result = self.client_results[service].popleft()

        r = rospy.Rate(1)
        while result["time"] < call_time and not rospy.is_shutdown():
            rospy.loginfo(f"got old message length ({len(result['data'])})")
            if len(self.client_results[service]) > 0:
                result = self.client_results[service].popleft()
            r.sleep()

        if len(result["data"]) < 500:
            rospy.loginfo(f"result is {result['data']}")
        rospy.loginfo(
            f"Recieved result message length ({len(result['data'])}) from service {service}"
        )
        # self._result_callback({"do_action": True, "message": result["data"]})
        return result["data"]


def main():
    # TODO this should be a rosparam
    #pattern_name = "dialogue"
    pattern_name = "multiple-choice"
    #trigger_intent = rospy.get_param("/input_test_" + pattern_name + "/")
    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + f"/pattern_scripting/{pattern_name}.json"

    with open(pattern_script_path, "r") as read_file:
        script = json.load(read_file)

    try:
        rospy.init_node(pattern_name)
        # Initialize the pattern with pattern sequence/loop
        dp = SequentialPattern(pattern_name, script)
        rospy.loginfo(f"Set up. Starting first step of {pattern_name} pattern.")
        dp.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
