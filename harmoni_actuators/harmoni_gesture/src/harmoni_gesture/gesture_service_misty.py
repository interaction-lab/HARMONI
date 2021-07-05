#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State, ActuatorNameSpace
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from std_msgs.msg import String, Bool
import numpy as np
import ast
import requests
from requests.exceptions import Timeout
import xml.etree.ElementTree as ET

class GestureService(HarmoniServiceManager):
    """
    Gesture service
    """

    def __init__(self, name, param):
        """ Initialization of variables and gesture parameters """
        super().__init__(name)
        self.gestures_name = []
        self.gestures_duration = []
        self.gesture_list_received = False
        self.gesture_done = False
        self.name = name
        self.service_id = hf.get_child_id(self.name)
        #TODO cancellare non dovrebbe servire
        self.gesture_pub = rospy.Publisher(
            ActuatorNameSpace.gesture.value + self.service_id, String, queue_size=1
        )
        #TODO cancellare non dovrebbe servire
        self.gesture_sub = rospy.Subscriber(
            ActuatorNameSpace.gesture.value + self.service_id + "/get_list",
            String,
            self._get_list_callback,
            queue_size=1,
        )
        #TODO cancellare non dovrebbe servire
        self.gesture_done_sub = rospy.Subscriber(
            ActuatorNameSpace.gesture.value + self.service_id + "/done",
            Bool,
            self._gesture_done_callback,
            queue_size=1,
        )
        self.state = State.INIT
        self.setup_gesture()
        self.path = param["path"]
        return

    def _gesture_done_callback(self, data):
        """Callback function for gesture done

        Args:
            data (bool): gesture done (True)
        """
        if data:
            self.gesture_done = True

    def _get_list_callback(self, data):
        """Getting the list from the gesture reader

        Args:
            data (str): json of items of gesture for a specific robot 
        """
        print("List callback")
        if self.gestures_name == []:
            rospy.loginfo("Gesture list received")
            data = ast.literal_eval(data.data)
            for item in data:
                self.gestures_name.append(item["name"])
                self.gestures_duration.append(item["duration"])
            self.gesture_list_received = True

    def setup_gesture(self):
        """ Setup the gesture """
        rospy.loginfo("Setting up the %s" % self.name)
        #while not self.gesture_list_received:
        #    rospy.sleep(0.1)
        rospy.loginfo("Received list of gestures")
        # self._get_list_callback("{'name':'QT/point_front', 'duration':'4'}")
        return

    def do(self, data):
        """Do the gesture

        Args:
            data (str): it could be a string of:
             - object containing {"behavior_data": str} (as results of the TTS synthetization)
             - object of: {"name": str, "timing": int}

        Returns:
            response (int): state of the DO action
        """
        self.state = State.REQUEST
        self.actuation_completed = False
        if type(data) == str:
            data = ast.literal_eval(data)
        try:
            rospy.loginfo(f"length of data is {len(data)}")
            if "behavior_data" in data:
                data = ast.literal_eval(data["behavior_data"])
                gesture_data = self._get_gesture_data(data)
            else:
                gesture_data = data
                self.gesture_pub.publish(str(data))
            print(gesture_data)
            if gesture_data:
                while not self.gesture_done:
                    self.state = State.REQUEST
            self.state = State.SUCCESS
            self.gesture_done = False
            self.actuation_completed = True
        except IOError:
            rospy.logwarn("Gesture failed")
            self.state = State.FAILED
            self.actuation_completed = True
        return {"response": self.state}

    def request(self, data):
        """Request to the robot to do the gesture

        Args:
            data (str): it could be a string of:
             - object containing {"behavior_data": str} (as results of the TTS synthetization)
             - object of: {"name": str, "timing": int}

        Returns:
            response (int): state of the REQUEST action
        """
        self.state = State.REQUEST
        self.actuation_completed = False

        #if type(data) == str:
        #    data = ast.literal_eval(data)

        try:
            rospy.loginfo(f"length of data is {len(data)}")
            if "behavior_data" in data:
                data = ast.literal_eval(data["behavior_data"])
                gesture_data = self._get_gesture_data(data)
            else:
                gesture_data = data
                #self.gesture_pub.publish(str(data))

            print(gesture_data)
            if gesture_data:
                self.parse_gesture_misty(gesture_data)
                while not self.gesture_done:
                    self.state = State.REQUEST
            self.state = State.SUCCESS
            self.gesture_done = False
            self.actuation_completed = True
            self.response_received = True
        except Timeout:
            rospy.logwarn("Gesture failed: The ip of the robot appears unreachable")
            self.state = State.FAILED
            self.actuation_completed = True
            self.response_received = False
        return {"response": self.state}

    def parse_gesture_misty(self, data):
        tree = ET.parse(self.path + "/" + ast.literal_eval(data)["gesture"] + ".xml")
        root = tree.getroot()
        current_time = 0.0
        for child in root.findall("waypoints/point"):
            gesture_type = child.find("gesture_type").text
            time = float(child.find("Time").text)
            if (current_time < time):
                #print((time-current_time))
                rospy.sleep((time - current_time))
                current_time = time
            if (gesture_type == "move_head"):
                url, payload = self.move_head(int(child.find("Pitch").text), int(child.find("Roll").text), int(child.find("Yaw").text))
            elif gesture_type == 'move_arms':
                url, payload = self.move_arms(int(child.find("LeftArmPosition").text), int(child.find("RightArmPosition").text))
            else:
                #throw_exception 
                url, payload = None, None
                pass
            print(url)
            print(payload)
            try:
                response = requests.post(url, 
                    params = payload,
                    timeout = 1)
            except Timeout:
                response = requests.post(url, 
                    params = payload,
                    timeout = 4)
                rospy.logwarn("Gesture failed: The ip of the robot appears unreachable")
            print(response)
        self.gesture_done = True

    def move_head(self, Pitch=0, Roll=0, Yaw=0):
        payload = {'Pitch': Pitch, 
                    'Roll': Roll,
                    'Yaw': Yaw
        }
        print("Sending request")
        url = 'http://{}/api/head'.format(rospy.get_param("/robot_ip"))
        return url, payload

    def move_arms(self, LeftArmPosition = None, RightArmPosition = None):
        payload = {'LeftArmPosition': LeftArmPosition, 
                    'RightArmPosition': RightArmPosition
        }
        print("Sending request")
        url = 'http://{}/api/arms/set'.format(rospy.get_param("/robot_ip"))
        return url, payload

    def _get_gesture_data(self, data):
        """Getting the gesture data parsing the output of TTS

        Args:
            data (str): string of json {"behavior_data":str}

        Returns:
            [bool]: getting the gesture done (True)
        """
        if type(data) == str:
            data = ast.literal_eval(data)
        behavior_data = ast.literal_eval(data["behavior_data"])
        words_data = list(filter(lambda b: b["type"] == "word", behavior_data))
        behavior_set = []
        sentence = []
        for b in behavior_data:
            if "id" in b.keys():
                if b["id"] in self.gestures_name:
                    behavior_set.append(b)
            if "character" in b.keys():
                sentence.append(b["value"])
        ordered_gesture_data = list(
            sorted(behavior_set, key=lambda face: face["start"])
        )
        print(ordered_gesture_data)
        validated_gesture = []
        for gest in ordered_gesture_data:
            validated_gesture.append(gest["id"])
        if ordered_gesture_data == []:
            rospy.loginfo("No gestures")
            return False
        timing_word_behaviors = words_data + ordered_gesture_data
        ordered_timing_word_behaviors = list(
            sorted(timing_word_behaviors, key=lambda behavior: behavior["start"])
        )
        start_time = rospy.Time.now()
        for index, behav in enumerate(ordered_timing_word_behaviors[:-1]):
            print(ordered_timing_word_behaviors[index])
            if behav["type"] != "word":
                print("Here")
                while rospy.Time.now() - start_time < rospy.Duration.from_sec(
                    behav["start"]
                ):
                    pass
                gesture_timing = float(
                    ordered_timing_word_behaviors[index + 1]["start"]
                )  # you cannot have a behavior sets at the end of the sentence
                rospy.loginfo(
                    "Play "
                    + str(behav["id"])
                    + " at time:"
                    + str(behav["start"])
                    + " with a duration of: "
                    + str(gesture_timing)
                )
                data = {"gesture": behav["id"], "timing": gesture_timing}
                self.gesture_pub.publish(str(data))

        if ordered_timing_word_behaviors[len(ordered_timing_word_behaviors) - 1]:
            if (
                ordered_timing_word_behaviors[len(ordered_timing_word_behaviors) - 1][
                    "type"
                ]
                != "word"
            ):
                print("Here")
                while rospy.Time.now() - start_time < rospy.Duration.from_sec(
                    ordered_timing_word_behaviors[
                        len(ordered_timing_word_behaviors) - 1
                    ]["start"]
                ):
                    pass
                gesture_timing = float(
                    ordered_timing_word_behaviors[
                        len(ordered_timing_word_behaviors) - 1
                    ]["start"]
                )  # you cannot have a behavior sets at the end of the sentence
                rospy.loginfo(
                    "Play "
                    + str(
                        ordered_timing_word_behaviors[
                            len(ordered_timing_word_behaviors) - 1
                        ]["id"]
                    )
                    + " at time:"
                    + str(
                        ordered_timing_word_behaviors[
                            len(ordered_timing_word_behaviors) - 1
                        ]["start"]
                    )
                    + " with a duration of: "
                    + str(gesture_timing)
                )
                data = {
                    "gesture": ordered_timing_word_behaviors[
                        len(ordered_timing_word_behaviors) - 1
                    ]["id"],
                    "timing": gesture_timing,
                }
                self.gesture_pub.publish(str(data))
        return True


def main():
    service_name = ActuatorNameSpace.gesture.name
    instance_id = rospy.get_param("/instance_id")
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name)
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = GestureService(service_name, params)
        service_server = HarmoniServiceServer(service_id, s)
        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
