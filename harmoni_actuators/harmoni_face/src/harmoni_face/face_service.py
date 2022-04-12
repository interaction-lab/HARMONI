#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
from harmoni_face.msg import FaceRequest
from harmoni_face.face_client import Face
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from threading import Timer
from collections import deque
import json
import ast
import os

class NoseService(HarmoniServiceManager):
    """
    Nose service
    """

    def __init__(self, name, param_nose, face):
        """Init of the nose service and setup as Harmoni service
        Args:
            name (str): service name
            param (obj): list of parameters useful for the service
                gaze_speed (int): velocity of the gaze command (milliseconds)
        """
        super().__init__(name)
        self.name = name
        self.service_id = hf.get_child_id(self.name)
        self.face = face
        if not self.face.connected:
            self.face.setup_ros()
        self.face_nose_pub = rospy.Publisher(
            ActuatorNameSpace.face.value + self.service_id + "/expressing/nose",
            FaceRequest,
            queue_size=1,
        )
        self.state = State.INIT
        return

    def do(self, data):
        """Do expression of nose in web face
        Args:
            data ([str]): stringified json from tts results  
        Returns:
            response (int): whether SUCCESS of FAIL
            message (str): result message 
        """
        self.actuation_completed = False
        try:
            self.state = State.REQUEST
            face_bool = self.face.face_sequential_request(data)
            rospy.loginfo("Completed Expression")
            self.state = State.SUCCESS
            self.actuation_completed = True
            self.result_msg=""
        except Exception:
            self.state = State.FAILED
            self.actuation_completed = True
            self.result_msg=""
        return {"response": self.state, "message": self.result_msg}

    def send_face_nose_request(self):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage of the face eyes")
        self.face_nose_pub.publish(self.face_request_nose)
        return


class EyesService(HarmoniServiceManager):
    """
    Eyes service
    """

    def __init__(self, name, param_eyes, face):
        """Init of the Eyes service and setup as Harmoni service
        Args:
            name (str): service name
            param (obj): list of parameters useful for the service
                gaze_speed (int): velocity of the gaze command (milliseconds)
        """
        super().__init__(name)
        self.name = name
        self.face = face
        if not self.face.connected:
            self.face.setup_ros()
        self.gaze_speed = param_eyes["gaze_speed"]
        self.timer_interval = param_eyes["timer_interval"]
        self.service_id = hf.get_child_id(self.name)
        self.face_eyes_pub = rospy.Publisher(
            ActuatorNameSpace.face.value + self.service_id + "/expressing/eyes",
            FaceRequest,
            queue_size=1,
        )
        self.state = State.INIT
        return

    def do(self, data):
        """Do expression of eyes in web face
        Args:
            data ([str]): stringified json from tts results  
        Returns:
            response (int): whether SUCCESS of FAIL
            message (str): result message 
        """
        self.actuation_completed = False
        [gaze_bool, gaze_data] = self._get_gaze_data(data)
        try:
            self.state = State.REQUEST
            # The gaze eyes are played first
            if gaze_bool:
                rospy.loginfo("Valid gaze")
                rospy.loginfo(f"The valid gaze is {gaze_data}")
                self.face_request_eyes = FaceRequest(
                                retarget_gaze=gaze_bool, gaze_target=gaze_data, hold_gaze=2
                            )
                t = Timer(self.timer_interval, self.send_face_eyes_request)
                t.start()
                start_time = rospy.Time.now()
            face_bool = self.face.face_sequential_request(data)
            rospy.loginfo("Completed Expression")
            self.state = State.SUCCESS
            self.actuation_completed = True
            self.result_msg=""
        except Exception:
            self.state = State.FAILED
            self.actuation_completed = True
            self.result_msg=""
        return {"response": self.state, "message": self.result_msg}

    def send_face_eyes_request(self):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage of the face eyes")
        self.face_eyes_pub.publish(self.face_request_eyes)
        return

    def _get_gaze_data(self, data):
        """Get target gaze data"""
        data = ast.literal_eval(data)
        if "behavior_data" in data:
            behavior_data = ast.literal_eval(data["behavior_data"])
        else:
            behavior_data = data
        gaze_bool = False
        gaze_target = Point()
        for b in behavior_data:
            if "id" in b.keys():
                if b["id"] == "target":
                    gaze_target.x = b["point"][0]
                    gaze_target.y = b["point"][1]
                    gaze_target.z = b["point"][2]
                    gaze_bool = True
        return gaze_bool, gaze_target

    
class MouthService(HarmoniServiceManager):
    """
    Mouth service
    """

    def __init__(self, name, param_mouth, face):
        """Init of the Mouth service and setup as Harmoni service
        Args:
            name (str): service name
            param_mouth (obj): list of parameters useful for the service
                min_duration_viseme (int): minimum amount of time a viseme can last (seconds)
                speed_viseme (int): velocity of the viseme execution (milliseconds)
                timer_interval (int): interval of time between two consecutive visemes (seconds)
        """
        super().__init__(name)
        rospy.loginfo("MouthService initializing")
        self.name = name
        self.face = face
        if not self.face.connected:
            self.face.setup_ros()
        self.min_duration_viseme = param_mouth["min_duration_viseme"]
        self.speed_viseme = param_mouth["speed_viseme"]
        self.timer_interval = param_mouth["timer_interval"]
        self.service_id = hf.get_child_id(self.name)
        self.face_mouth_pub = rospy.Publisher(
            ActuatorNameSpace.face.value + self.service_id + "/expressing/mouth",
            FaceRequest,
            queue_size=1,
        )
        self.state = State.INIT
        return

    def do(self, data):
        """Do expression in web face
        Args:
            data ([str]): stringified json from tts results  
        Returns:
            response (int): whether SUCCESS of FAIL
            message (str): result message 
        """
        rospy.loginfo("Do expressions")
        self.actuation_completed = False
        self.result_msg=""
        [viseme_bool, visemes] = self._get_viseme_data(data)
        try:
            self.state = State.REQUEST
            # The visemes are played at first
            if viseme_bool:
                viseme_ids = list(map(lambda b: b["id"], visemes))
                viseme_times = list(map(lambda b: b["start"], visemes))
                self.face_mouth_request = FaceRequest(
                    visemes=viseme_ids, viseme_ms=self.speed_viseme, times=viseme_times
                )
                t = Timer(self.timer_interval, self.send_face_mouth_request)
                t.start()
                start_time = rospy.Time.now()
                rospy.loginfo("The last viseme lasts %i" % viseme_times[-1])
                time_sleep = int(viseme_times[-1]) + self.min_duration_viseme
                rospy.sleep(time_sleep)
            face_bool = self.face.face_sequential_request(data)
            self.state = State.SUCCESS
            self.actuation_completed = True
            rospy.loginfo("Completed Expression")
        except Exception:
            self.state = State.FAILED
            self.actuation_completed = True
        return {"response": self.state, "message": self.result_msg}


    def send_face_mouth_request(self):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage of the face")
        self.face_mouth_pub.publish(self.face_mouth_request)
        return

    def _get_viseme_data(self, data):
        """ Get the validated data of the face"""
        viseme_bool = False
        data = ast.literal_eval(data)
        if "behavior_data" in data:
            behavior_data = ast.literal_eval(data["behavior_data"])
        else:
            behavior_data = data
        viseme_set = []
        for b in behavior_data:
            if "id" in b.keys():
                if b["id"] in self.face.visemes:
                    viseme_set.append(b)
                    viseme_bool = True
        if viseme_bool:
            for i in range(0, len(viseme_set) - 1):
                viseme_set[i]["duration"] = (
                    viseme_set[i + 1]["start"] - viseme_set[i]["start"]
                )

            viseme_set[-1]["duration"] = self.min_duration_viseme
            viseme_behaviors = list(
                filter(lambda b: b["duration"] >= self.min_duration_viseme, viseme_set)
            )
            ordered_visemes = list(sorted(viseme_behaviors, key=lambda b: b["start"]))
            rospy.loginfo("The validated visemes are %s" % viseme_set)
        return viseme_bool, viseme_set

def main():
    """Set names, collect params, and give service to server"""
    service_name = ActuatorNameSpace.face.name
    instance_id = rospy.get_param("/instance_id") #default
    service_id_mouth = f"{service_name}_mouth_{instance_id}"
    service_id_eyes = f"{service_name}_eyes_{instance_id}"
    service_id_nose = f"{service_name}_nose_{instance_id}"
    try:
        rospy.init_node(service_name)
        param = rospy.get_param(service_name + "/" + instance_id + "_param")
        param_eyes = rospy.get_param(service_name + "/" + instance_id + "_param/eyes/")
        param_mouth = rospy.get_param(service_name + "/" + instance_id + "_param/mouth/")
        param_nose = rospy.get_param(service_name + "/" + instance_id + "_param/nose/")
        face = Face(ActuatorNameSpace.face.value, instance_id, param)
        s_eyes = EyesService(service_name + "_eyes_" + instance_id, param_eyes, face)
        s_mouth = MouthService(service_name + "_mouth_" + instance_id, param_mouth, face)
        s_nose = NoseService(service_name + "_nose_" + instance_id, param_nose, face)
        service_server_eyes = HarmoniServiceServer(service_id_eyes, s_eyes)
        service_server_mouth = HarmoniServiceServer(service_id_mouth, s_mouth)
        service_server_nose = HarmoniServiceServer(service_id_nose, s_nose)

        print(service_name)
        print("****************************************************************************")
        print(service_id_eyes)
        print(service_id_mouth)
        print(service_id_nose)
        
        service_server_eyes.start_sending_feedback()
        service_server_mouth.start_sending_feedback()
        service_server_nose.start_sending_feedback()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()