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


class SequentialPattern(HarmoniServiceManager):
    """Plays through a sequence of steps described in the script json

    Script is made of a sequence of 'set' objects. Each set has a sequence of steps.
    The steps provide directions for an action request to a node. Steps can consist
    of a list of steps which will be executed in parallel.

    Set types include:
        'setup' which plays once at the start
        'sequence' which plays through once
        'loop'  which continues indefinitely

    Actions specify the service id (e.g. tts_default), and the following:
        'action_goal' the type of command to give to the service (e.g. DO, START, STOP, etc.)
        'resource_type' the type of server expected to provide the service
        'wait_for' the condition to wait for
        'trigger' the additional message to send to the service

    Results from services are stored in a single dictionary with the service
    name as the key and the results in a list. Each result is tagged with the time received
    and data recieved.

    Detections are gathered with individual callbacks to each detector's topic and stored with
    the results.

    """

    def __init__(self, name, script):
        super().__init__(name)
        """Init the behavior pattern and setup the clients"""
        self.script = script
        self.end_pattern = False  # Variable for interupting the script
        self.scripted_services = set()  # services used in this script
        self.script_set_index = 0

        self.scripted_services = self._get_services(script)
        self._setup_clients()

        if script[self.script_set_index]["set"] == "setup":
            self.setup_services(script[self.script_set_index]["steps"])
            self.script_set_index += 1

        self.state = State.INIT
        return

    def _setup_clients(self):
        """Set up clients to all services that have been scripted

        Also checks that the service matches what has been specified in the
        decision configuration file.
        """
        list_repos = hf.get_all_repos()
        for repo in list_repos:
            [_, child_list] = hf.get_service_list_of_repo(repo)
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
            f"{self.name} Pattern requires these services: {self.scripted_services}"
        )

        for name, client in self.service_clients.items():
            client.setup_client(name, self._result_callback, self._feedback_callback)
        rospy.loginfo("Behavior interface action clients have been set up!")
        return

    def _get_services(self, script):
        """Extract all the services used in a given script.

        Args:
            script (list of dicts): list of sets of actions that

        Returns:
            list: names of all the services
        """
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
        """ Feedback is currently just logged """
        rospy.logdebug("The feedback recieved is %s." % feedback)
        # Check if the state is end, stop the behavior pattern
        # if feedback["state"] == State.END:
        #    self.end_pattern = True
        return

    def _detecting_callback(self, data, service_name):
        """Store data from detection to client_results dictionary"""
        data = data.data
        self.client_results[service_name].append({"time": time(), "data": data})
        return

    def start(self):
        """Iterate through steps of the script until reaching the end."""
        self.state = State.START
        r = rospy.Rate(1)
        while self.script_set_index < len(self.script) and not rospy.is_shutdown():
            # If scripts were not setup in the init, they will be here
            rospy.loginfo("Running the following steps:")
            rospy.loginfo(self.script[self.script_set_index]["steps"])
            if self.script[self.script_set_index]["set"] == "setup":
                self.setup_services(self.script[self.script_set_index]["steps"])

            elif self.script[self.script_set_index]["set"] == "sequence":
                # self.count = -1
                print("good")
                self.do_steps(self.script[self.script_set_index]["steps"])

            elif self.script[self.script_set_index]["set"] == "loop":
                # self.count = -1
                self.do_steps(self.script[self.script_set_index]["steps"], looping=True)
            elif self.end_pattern:
                # for client in self.scripted_services:
                #    self.stop(client)
                break
            self.script_set_index += 1
            r.sleep()
        return self.result

    def stop(self):
        """Stop the Pattern Player """
        try:
            for _, client in self.service_clients.items():
                client.cancel_goal()
            self.state = State.SUCCESS
        except Exception as E:
            self.state = State.FAILED
        return

    def pause(self):
        """Pause the Behavior Pattern """
        # TODO: implement a pause
        return

    def setup_services(self, setup_steps):
        """Setup sensor and detector services

        Sensors and detectors are directed to turn 'ON' and a callback
        is created for detectors.

        Args:
            setup_steps (list of dicts): call to each sensor/detector to set up.
        """
        for d in setup_steps:
            for service, details in d.items():

                assert details["resource_type"] in [
                    "sensor",
                    "detector",
                ], "Can only set up sensors or detectors"

                # Send request for each sensor service to set themselves up
                self.service_clients[service].send_goal(
                    action_goal=ActionType[details["action_goal"]].value,
                    optional_data="Setup",
                    wait=details["wait_for"],
                )

                if details["resource_type"] == "detector":
                    # Split off last part of node name to get the topic (e.g. stt_default -> stt)
                    service_list = service.split("_")
                    service_id = "_".join(service_list[0:-1])

                    topic = f"/harmoni/detecting/{service_id}/{service_list[-1]}"

                    rospy.loginfo(f"subscribing to topic: {topic}")

                    rospy.Subscriber(
                        topic,
                        String,
                        self._detecting_callback,
                        callback_args=service,
                        queue_size=1,
                    )
        return

    def do_steps(self, sequence, looping=False):
        """Directs the services to do each of the steps scripted in the sequence

        Args:
            sequence (list of dicts): Each dict specifies a call to a service
            looping (bool, optional): If true will loop the sequence indefinitely. Defaults to False.
        """

        passthrough_result = None
        for cnt, step in enumerate(sequence, start=1):
            if rospy.is_shutdown():
                return
            rospy.loginfo(f"------------- Starting sequence step: {cnt}-------------")

            if passthrough_result:
                rospy.loginfo(f"with prior result length ({len(passthrough_result)})")

            else:
                rospy.loginfo("no prior result")

            passthrough_result = self.handle_step(step, passthrough_result)

            rospy.loginfo(f"************* End of sequence step: {cnt} *************")
            #print(f"passthrough result:{passthrough_result}")
        self.result = passthrough_result
        if looping:
            rospy.loginfo("Done with a loop!")
            if not rospy.is_shutdown():
                self.do_steps(sequence, looping=True)

        return

    def handle_step(self, step, optional_data=None):
        """Handle cases for different types of steps

        Handles:
            parallel execution of list steps
            erronious calls to sensors
            pulling latest data from detectors
            making requests of actuators or other services

        Args:
            step ([type]): [description]
            optional_data ([type], optional): [description]. Defaults to None.

        Returns:
            [type]: [description]
        """
        # If it is an array, it means that is a parallel actions, so I start multiple goals
        # In the current implementation parallel actions return values will not get passed on
        # TODO modify to collect results and return them
        if isinstance(step, list):
            threads = []
            rospy.loginfo("Running action in parallel-ish (launching multiple goals)")
            for i, sub_action in enumerate(step, start=1):
                t = threading.Thread(
                    target=self.handle_step, args=(sub_action, optional_data)
                )
                threads.append(t)
                t.start()
                # result = self.handle_step(sub_action, optional_data)
            for t in threads:
                t.join()
            result = None
            return result

        else:
            service = next(iter(step))
            details = step[service]
            rospy.loginfo(f"Step is {service} with details {details}")
            assert details["resource_type"] in [
                "sensor",
                "detector",
                "actuator",
                "service",
            ], "must specify resource type of each step"

            self.state = State.REQUEST

            if details["resource_type"] == "sensor":
                rospy.logwarn("Sensor should be set up during init")
                result = None

            elif details["resource_type"] == "detector":
                return_data = self.make_detector_request(service, details)

            else:
                return_data = self.make_service_request(service, details, optional_data)
        return return_data

    def make_service_request(self, service, details, optional_data):
        """Sends a goal to a service

        Args:
            service (str): Name of the service
            details (dict): goal details
            optional_data (str): can be either the prior result or the trigger from the script

        Returns:
            str: the result of the request
        """
        # The trigger has priority to be passed through, then the prior result
        # if neither are set it will be left as ""
        if "trigger" in details.keys():
            optional_data = details["trigger"]
        elif not optional_data:
            optional_data = ""

        rospy.loginfo(
            f"Sending goal to {service} optional_data len {len(optional_data)}"
        )

        # The request will be made without waiting as the get_new_result function
        # can handle the waiting
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

    def make_detector_request(self, service, details):
        """Get detection result from detector service

        Args:
            service (str): Name of the detector service
            details (dict): dictionary of request details. 'wait_for' is the only relevant item

        Returns:
            str: the string version of the last detection
        """
        rospy.loginfo(f"Retrieving data from detector: {service}")
        if details["wait_for"] == "new":
            return_data = self.get_new_result(service)
        else:
            rospy.logwarn("Not waiting for a detector may return old result")
            if len(self.client_results[service]) > 0:
                return_data = self.client_results[service].popleft()["data"]
            return_data = None
        return return_data

    def get_new_result(self, service):
        """Waits for a new result for the service to be set

        Args:
            service (str): Name of the service

        Returns:
            str: Result data
        """
        rospy.loginfo("getting result from the service")
        rospy.loginfo(f"Queue is {self.client_results[service]}")
        rospy.loginfo(f"Queue size is {len(self.client_results[service])}")

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
        #print(f"result : {result}" )
        return result["data"]

    def reset_init(self):
        self.script_set_index = 0
        self.end_pattern = False
        for client in self.scripted_services:
            self.client_results[client] = deque()
        self.state = State.INIT

    def request(self, data):
        """Send goal request to appropriate child"""
        rospy.loginfo("Start the %s request" % self.name)
        rospy.loginfo(data)
        if isinstance(data,str):
           data = ast.literal_eval(data)
        self.script = data
        self.state = State.REQUEST
        r = rospy.Rate(10)
        while self.script_set_index < len(self.script) and not rospy.is_shutdown():
            if self.script[self.script_set_index]["set"] == "setup":
                self.setup(self.script[self.script_set_index]["steps"])
            elif self.script[self.script_set_index]["set"] == "sequence":
                self.count = -1
                self.do_steps(self.script[self.script_set_index]["steps"])
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
        return self.result




def main():
    """Set names, collect params, and give service to server"""

    call_start = rospy.get_param("start")
    pattern_to_use = rospy.get_param("pattern_name")
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{pattern_to_use}_{instance_id}"

    rospack = rospkg.RosPack()
    pck_path = rospack.get_path("harmoni_pattern")
    pattern_script_path = pck_path + f"/pattern_scripting/{pattern_to_use}.json"
    with open(pattern_script_path, "r") as read_file:
        script = json.load(read_file)

    try:
        rospy.init_node(pattern_to_use, log_level=rospy.INFO)

        # multiple_choice/default_param/[all your params]
        params = rospy.get_param(pattern_to_use + "/" + instance_id + "_param/")

        s = SequentialPattern(pattern_to_use, script)
        if call_start:
            s.start()
        else:
            service_server = HarmoniServiceServer(service_id, s)
            service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # pattern_to_use = rospy.get_param("/pattern_name/")
    # test = rospy.get_param("/test_" + pattern_to_use + "/")
    # # test_input = rospy.get_param("/test_input_" + pattern_to_use + "/")
    # instance_id = rospy.get_param("/instance_id_" + pattern_to_use + "/")
    # # trigger_intent = rospy.get_param("/test_input_" + pattern_to_use + "/")

    # try:
    #     rospy.init_node(pattern_to_use)
    #     # Initialize the pattern with pattern sequence/loop
    #     dp = SequentialPattern(pattern_to_use, script)
    #     service_server = HarmoniServiceServer(name=pattern_to_use, service_manager=dp)
    #     if test:
    #         rospy.loginfo(
    #             f"START: Set up. Testing first step of {pattern_to_use} pattern."
    #         )
    #         dp.start()
    #     else:
    #         service_server.start_sending_feedback()
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass


if __name__ == "__main__":
    main()
