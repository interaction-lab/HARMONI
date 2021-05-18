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
from geometry_msgs.msg import Point
from threading import Timer
from collections import deque
import json
import ast
import os


class EyesService(HarmoniServiceManager):
    """
    Eyes service
    """

    def __init__(self, name, param_eyes, param_mouth):
        """Init of the Eyes service and setup as Harmoni service

        Args:
            name (str): service name
            param (obj): list of parameters useful for the service
                gaze_speed (int): velocity of the gaze command (milliseconds)
        """
        super().__init__(name)
        self.name = name
        self.gaze_speed = param_eyes["gaze_speed"]
        self.service_id = hf.get_child_id(self.name)
        self.mouth = MouthService("face_mouth_default", param_mouth)
        self.timer_interval = self.mouth.timer_interval_expressions
        self.setup_face()
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
        face_expressions = self.mouth.get_facial_expression_data(data)
        [au_bool, aus] = self._get_face_eyes_data(data)
        [gaze_bool, gaze_data] = self._get_gaze_data(data)
        if au_bool:
            face_expressions.append(aus)
        try:
            self.state = State.REQUEST
            if gaze_bool:
                rospy.loginfo("Valid gaze")
                rospy.loginfo(f"The valid gaze is {gaze_data}")
                self.face_request_eyes = FaceRequest(
                                retarget_gaze=gaze_bool, gaze_target=gaze_data, hold_gaze=2
                            )
                self.send_face_eyes_request()
            if face_expressions != []:
                rospy.loginfo("Valid face expression or action unit not null")
                for face_expression in face_expressions:
                    if len(face_expression) > 1:
                        for ind in range(0,len(face_expression)-1):
                            f = face_expression[ind]
                            rospy.loginfo("The valid expression is %s" % f)
                            aus = list(map(lambda s: s[2:], f["aus"]))
                            au_ms = f["au_ms"] * 1000
                            au_degrees = f["au_degrees"]
                            self.face_request_eyes = FaceRequest(
                                aus=aus, au_degrees=au_degrees, au_ms=int(au_ms)
                            )
                            self.send_face_eyes_request()
                            rospy.sleep(self.timer_interval) 
                    aus = list(map(lambda s: s[2:], face_expression[-1]["aus"]))
                    au_ms = face_expression[-1]["au_ms"] * 1000
                    self.face_request_eyes = FaceRequest(
                        aus=aus,
                        au_degrees=face_expression[-1]["au_degrees"],
                        au_ms=int(au_ms),
                    )
                    self.send_face_eyes_request()
                    rospy.sleep(self.timer_interval) 
                    rospy.loginfo("The last facial expression ends")
            self.state = State.SUCCESS
            self.actuation_completed = True
            self.result_msg=""
        except Exception:
            self.state = State.FAILED
            self.actuation_completed = True
            self.result_msg=""
        rospy.loginfo("Completed Expression")
        return {"response": self.state, "message": self.result_msg}

    def setup_face(self):
        """ Setup the face """
        rospy.loginfo("Setting up the %s eyes" % self.name)
        rospy.loginfo("Checking that face is connected to ROS websocket")
        while not self.mouth.connected:
            rospy.loginfo("Wait for connection with face")
            rospy.sleep(1)
        rospy.loginfo("Done, face of eyes is connected to ROS websocket")
        [
            self.face_expression,
            self.face_expression_names,
        ] = self.mouth.get_facial_expressions_list()
        self.action_units = [ "au1", "au2", "au5", "au12", "au17", "au43", "au4", "au7", "au10", "au18", "au20", "au23", "au27", "au13", "au11"]
        return

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

    def _get_face_eyes_data(self, data):
        """ Get the validated data of the face"""
        data = ast.literal_eval(data)
        au_bool = False
        if "behavior_data" in data:
            behavior_data = ast.literal_eval(data["behavior_data"])
        else:
            behavior_data = data
        action_units_set =[]
        for b in behavior_data:
            if "id" in b.keys():
                if b["id"] in self.action_units:
                    action_units_set.append(b)
        ordered_au_data = list(
            sorted(action_units_set, key=lambda face: face["start"])
        )
        validated_au_expr = []
        for fexp in ordered_au_data:
            if fexp["id"] in self.action_units:
                validated_au_expr.append({
                        "aus": [fexp["id"]],
                        "au_degrees": [fexp["pose"]],
                        "au_ms": int(fexp["time"]),
                    })
        rospy.loginfo("The validated action units are %s" % validated_au_expr)
        if validated_au_expr!=[]:
            au_bool = True
        return au_bool, validated_au_expr


class MouthService(HarmoniServiceManager):
    """
    Mouth service
    """

    def __init__(self, name, param):
        """Init of the Mouth service and setup as Harmoni service

        Args:
            name (str): service name
            param (obj): list of parameters useful for the service
                min_duration_viseme (int): minimum amount of time a viseme can last (seconds)
                speed_viseme (int): velocity of the viseme execution (milliseconds)
                timer_interval (int): interval of time between two consecutive visemes (seconds)
        """
        super().__init__(name)
        rospy.loginfo("MouthService initializing")
        self.name = name
        self.min_duration_viseme = param["min_duration_viseme"]
        self.speed_viseme = param["speed_viseme"]
        self.timer_interval = param["timer_interval"]
        self.timer_interval_expressions = param["timer_interval_expression"]
        self.connected = False
        self.service_id = hf.get_child_id(self.name)
        self.setup_face()
        self.face_pub = rospy.Publisher(
            ActuatorNameSpace.face.value + self.service_id + "/expressing",
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
        face_expressions = self.get_facial_expression_data(data)
        [viseme_bool, visemes] = self._get_viseme_data(data)
        try:
            self.state = State.REQUEST

            if viseme_bool:
                viseme_ids = list(map(lambda b: b["id"], visemes))
                viseme_times = list(map(lambda b: b["start"], visemes))
                self.face_request = FaceRequest(
                    visemes=viseme_ids, viseme_ms=self.speed_viseme, times=viseme_times
                )
                t = Timer(self.timer_interval, self.send_face_request)
                t.start()
                start_time = rospy.Time.now()
                rospy.loginfo("The last viseme lasts %i" % viseme_times[-1])
                time_sleep = int(viseme_times[-1]) + self.min_duration_viseme
                rospy.sleep(time_sleep)
            if face_expressions != []:
                rospy.loginfo("Valid face expression or action unit not null")
                for face_expression in face_expressions:
                    if len(face_expression) > 1:
                        for ind in range(0,len(face_expression)-1):
                            f = face_expression[ind]
                            rospy.loginfo("The valid expression is %s" % f)
                            aus = list(map(lambda s: s[2:], f["aus"]))
                            au_ms = f["au_ms"] * 1000
                            au_degrees = f["au_degrees"]
                            print(aus, au_ms, au_degrees)
                            self.face_request = FaceRequest(
                                aus=aus, au_degrees=au_degrees, au_ms=int(au_ms)
                            )
                            self.send_face_request()
                            rospy.sleep(self.timer_interval_expressions) 
                    aus = list(map(lambda s: s[2:], face_expression[-1]["aus"]))
                    au_ms = face_expression[-1]["au_ms"] * 1000
                    self.face_request = FaceRequest(
                        aus=aus,
                        au_degrees=face_expression[-1]["au_degrees"],
                        au_ms=int(au_ms),
                    )
                    self.send_face_request()
                    rospy.sleep(self.timer_interval_expressions) 
                    rospy.loginfo("The last facial expression ends")
            self.state = State.SUCCESS
            self.actuation_completed = True
        except Exception:
            self.state = State.FAILED
            self.actuation_completed = True
        rospy.loginfo("Completed Expression")
        return {"response": self.state, "message": self.result_msg}

    def setup_face(self):
        """Setup the face, waiting for the connection with the web page
        """
        rospy.loginfo("Setting up the %s mouth" % self.name)
        rospy.loginfo("Checking that face is connected to ROS websocket")
        rospy.wait_for_service("/harmoni/actuating/face/is_connected")
        rospy.loginfo("Done, face is connected to ROS websocket")
        self.connected = True
        [
            self.face_expression,
            self.face_expression_names,
        ] = self.get_facial_expressions_list()
        self.visemes = [
            "BILABIAL",
            "LABIODENTAL",
            "INTERDENTAL",
            "DENTAL_ALVEOLAR",
            "POSTALVEOLAR",
            "VELAR_GLOTTAL",
            "CLOSE_FRONT_VOWEL",
            "OPEN_FRONT_VOWEL",
            "MID_CENTRAL_VOWEL",
            "OPEN_BACK_VOWEL",
            "CLOSE_BACK_VOWEL",
            "IDLE",
        ]
        return

    def send_face_request(self):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage of the face")
        self.face_pub.publish(self.face_request)
        return

    def get_facial_expressions_list(self):
        """ Get facial expression list from the resource file"""
        facial_expression_list = []
        face_expression_au = {}
        base_dir = os.path.dirname(__file__)
        with open(
            base_dir + "/resource/cordial_face_expression.json", "r"
        ) as json_file:
            data = json.load(json_file)
            for facial_expression in data:
                facial_expression_list.append(facial_expression)
                au_name = str(facial_expression)
                aus = []
                face_expression_au[au_name] = []
                for dofs in data[facial_expression]["dofs"]:
                    aus.append(str(dofs))
                for keyframe in data[facial_expression]["keyframes"]:
                    au_degrees = keyframe["pose"]
                    au_ms = keyframe["time"]
                    face_expression_au[au_name].append({
                        "aus": aus,
                        "au_degrees": au_degrees,
                        "au_ms": au_ms,
                    })
        return face_expression_au, facial_expression_list

    def get_facial_expression_data(self,data):
        data = ast.literal_eval(data)
        if "behavior_data" in data:
            behavior_data = ast.literal_eval(data["behavior_data"])
        else:
            behavior_data = data
        facial_expression = []
        for b in behavior_data:
            if "id" in b.keys():
                if b["id"] in self.face_expression_names:
                    facial_expression.append(b)
        rospy.loginfo("These facial expressions include %s" % facial_expression)
        ordered_facial_data = list(
            sorted(facial_expression, key=lambda face: face["start"])
        )
        validated_face_expr = []
        for fexp in ordered_facial_data:
            validated_face_expr.append(self.face_expression[fexp["id"]])
        rospy.loginfo("The validated facial expressions are %s" % validated_face_expr)
        return validated_face_expr

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
                if b["id"] in self.visemes:
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
    instance_id = rospy.get_param("/instance_id")
    service_id_mouth = f"{service_name}_mouth_{instance_id}"
    service_id_eyes = f"{service_name}_eyes_{instance_id}"
    try:
        rospy.init_node(service_name)
        param_eyes = rospy.get_param(service_name + "/" + instance_id + "_param/eyes/")
        param_mouth = rospy.get_param(service_name + "/" + instance_id + "_param/mouth/")
        s_eyes = EyesService(service_name + "_eyes_" + instance_id, param_eyes, param_mouth)
        s_mouth = MouthService(service_name + "_mouth_" + instance_id, param_mouth)
        service_server_eyes = HarmoniServiceServer(service_id_eyes, s_eyes)
        service_server_mouth = HarmoniServiceServer(service_id_mouth, s_mouth)
        service_server_eyes.start_sending_feedback()
        service_server_mouth.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
