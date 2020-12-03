#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
import rospkg
import json
import numpy as np
from std_msgs.msg import String
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import DetectorNameSpace, ActionType
from collections import deque
from time import time
import threading
import ast


class SequentialPattern(HarmoniServiceManager):
    """
    Dialoging pattern class
    """

    def __init__(self, name, script):
        super().__init__(name)
        """Init the behavior pattern and setup the clients"""
        self.script = script
        self.end_pattern = False
        self.scripted_services = set()  # services used in this script
        self.script_set_index = 0
        self.name = name
        self.scripted_services = self._get_services(script)
        self._setup_clients()
        if script[self.script_set_index]["set"] == "setup":
            self.setup(script[self.script_set_index]["steps"])
            self.script_set_index += 1

        self.state = State.INIT
        return

    def reset_init(self):
        self.script_set_index = 0
        self.end_pattern = False
        for client in self.scripted_services:
            self.client_results[client] = deque()
        self.state = State.INIT


    def _setup_clients(self):
        """
        Set up all clients that have been configured in the
        harmoni_decision module
        """
        list_repos = hf.get_all_repos()
        for repo in list_repos:
            [repo_child_list, child_list] = hf.get_service_list_of_repo(repo)
            for child in child_list:
                self.configured_services.extend(hf.get_child_list(child))

        for service in self.scripted_services:
            assert (
                service in self.configured_services
            ), f"Scripted service: {service}, is not listed among configured services: {self.configured_services}"

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
        """ Receive and store result with timestamp """
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
        # Check if the state is end, stop the behavior pattern
        # if feedback["state"] == State.END:
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
        r = rospy.Rate(10)
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

    def request(self, data):
        """Send goal request to appropriate child"""
        rospy.loginfo("Start the %s request" % self.name)
        rospy.loginfo(data)
        if isinstance(data,str):
            data = ast.literal_eval(data)
        self.state = State.REQUEST
        r = rospy.Rate(10)
        while self.script_set_index < len(self.script) and not rospy.is_shutdown():
            if self.script[self.script_set_index]["set"] == "setup":
                self.setup(self.script[self.script_set_index]["steps"])

            elif self.script[self.script_set_index]["set"] == "sequence":
                self.count = -1
                self.do_sequence(self.script[self.script_set_index]["steps"],data=data)

            elif self.script[self.script_set_index]["set"] == "loop":
                self.count = -1
                self.do_sequence(
                    self.script[self.script_set_index]["steps"], looping=True, data=data
                )
            #elif self.end_pattern:
                ##TODO
            #    break
            self.script_set_index += 1
            r.sleep()
        rospy.loginfo("_________SEQUENCE PATTERN END__________")
        prepared = [dict(zip(cl, self.client_results[cl])) for cl in self.client_results]
        j = json.dumps(prepared)
        self.result_msg = str(j)
        self.response_received = True
        self.actuation_completed = True
        self.state = State.SUCCESS
        return self.result_msg

    def stop(self, service):
        """Stop the Behavior Pattern """
        rospy.loginfo("___________________STOP____________________")
        try:
            rospy.loginfo("The service is " + service)
            self.service_clients[service].send_goal(
                    action_goal=ActionType.OFF,
                    optional_data="",
                    wait=False,
                )
            self.state = State.SUCCESS
        except Exception as E:
            self.state = State.FAILED
        return

    def pause(self):
        """Pause the Behavior Pattern """
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
                optional_data="Setup",
                wait=details["wait_for"],
            )

            if details["resource_type"] == "detector":
                # Split off last part of node name to get the topic (e.g. stt_default -> stt)
                service_list = service.split("_")
                service_id = "_".join(service_list[0:-1])

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

    def do_sequence(self, sequence, looping=False, data=None):
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
                result=data
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
                elif service in optional_data:
                    optional_data=optional_data[service]
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

        return return_data

    def get_new_result(self, service):
        rospy.loginfo("getting result from the service")
        rospy.loginfo(f"Queue size is {len(self.client_results[service])}")
        rospy.logdebug(f"Queue is {self.client_results[service]}")

        call_time = time()
        result = {"time": 0, "data": "_the_queue_is_empty"}

        if len(self.client_results[service]) > 0:
            result = self.client_results[service].popleft()

        r = rospy.Rate(10)
        while result["time"] < call_time and not rospy.is_shutdown():
            rospy.logdebug(f"got old message length ({len(result['data'])})")
            if len(self.client_results[service]) > 0:
                result = self.client_results[service].popleft()
            r.sleep()

        if len(result["data"]) < 500:
            rospy.loginfo(f"result is {result['data']}")
        rospy.loginfo(
            f"Received result message length ({len(result['data'])}) from service {service}"
        )
        self.client_results[service].appendleft({"time": result["time"], "data": result["data"]})
        # self._result_callback({"do_action": True, "message": result["data"]})
        return result["data"]


def main():
    pattern_name = rospy.get_param("/pattern_name/")
    test = rospy.get_param("/test_" + pattern_name + "/")
    test_input = rospy.get_param("/test_input_" + pattern_name + "/")
    test_id = rospy.get_param("/test_id_" + pattern_name + "/")
    # trigger_intent = rospy.get_param("/test_input_" + pattern_name + "/")
    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + f"/pattern_scripting/{pattern_name}.json"
    with open(pattern_script_path, "r") as read_file:
        script = json.load(read_file)
    try:
        rospy.init_node(pattern_name)
        # Initialize the pattern with pattern sequence/loop
        dp = SequentialPattern(pattern_name, script)
        service_server = HarmoniServiceServer(name=pattern_name, service_manager=dp)
        if test:
            rospy.loginfo(
                f"START: Set up. Testing first step of {pattern_name} pattern."
            )
            dp.start()
        else:
            service_server.update_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
