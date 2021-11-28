#!/usr/bin/env python3

# Common Imports
import rospy
import roslib
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace
from harmoni_face.msg import FaceRequest
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from threading import Timer
from collections import deque
import json
import ast
import os

class Face():

    def __init__(self, name, instance_id, param):
        """[summary]

        Args:
            name ([str]): path for the topic of the service including the instance_id (e.g., harmoni/actuating/face)
            instance_id (str): instance of the service (e.g., default)
            param (obj): parameters in the configuration file
                timer_interval : seconds for the duration of the timer
        """
        self.name =name
        self.instance_id = instance_id
        self.timer_interval = param["timer_interval"]
        self.setup_face()
        self.connected = False
            
        
    
    def setup_ros(self):
        """Setting up and initializing ros topic
        """
        self.face_pub = rospy.Publisher(
            self.name +  self.instance_id +"/expressing",
            FaceRequest, # facial expression + action unit request
            queue_size=1,
        )
        rospy.loginfo("Checking that face is connected to ROS websocket")
        rospy.wait_for_service(self.name + "/is_connected")
        rospy.loginfo("Done, face is connected to ROS websocket")
        self.connected=True
        return True


    def setup_face(self):
        """
            Setup the face
        """
        [
            self.face_expression,
            self.face_expression_names,
        ] = self._get_facial_expressions_list()
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
        action_units_eyes = ["au1", "au2", "au4", "au5", "au6", "au43"]
        action_units_nose = ["au9", "au38", "au39"]
        action_units_mouth = ["au10", "au12", "au13", "au14", "au15", "au16", "au17", "au18", "au20", "au23", "au24", "au25", "au26", "au27"]
        self.action_units = action_units_eyes + action_units_nose + action_units_mouth
        return

    def send_face_request(self):
        """ Send the request to the web page"""
        rospy.loginfo("Sending request to webpage of the face eyes")
        self.face_pub.publish(self.face_request_eyes)
        return

    def face_sequential_request(self, data):
        """[summary]

        Args:
            data ([json]): string of array of this object:
                start:
                time:

        Returns:
            face_bool (bool): True if facial expressions and/or action units were detected and acted, False if no facial expression or action units were found
        """
        [face_expr_bool,face_expressions] = self._get_face_data(data)
        if face_expr_bool:
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
                        self.send_face_request()
                        rospy.sleep(self.timer_interval) 
                aus = list(map(lambda s: s[2:], face_expression[-1]["aus"]))
                au_ms = face_expression[-1]["au_ms"] * 1000
                self.face_request_eyes = FaceRequest(
                    aus=aus,
                    au_degrees=face_expression[-1]["au_degrees"],
                    au_ms=int(au_ms),
                )
                self.send_face_request()
                rospy.sleep(self.timer_interval) 
                rospy.loginfo("The last facial expression ends")
        else:
            rospy.loginfo("No facial expressions or Action units detected")
        return face_expr_bool


    def _get_facial_expressions_list(self):
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

    def _get_facial_expression_data(self,data):
        data = ast.literal_eval(data)
        face_expr_bool = False
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
        return  validated_face_expr


    def _get_aus_data(self, data):
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
                        "au_ms": int(fexp["start"]/1000),
                    })
        rospy.loginfo("The validated action units are %s" % validated_au_expr)
        if validated_au_expr!=[]:
            au_bool = True
        return au_bool, validated_au_expr

    def _get_face_data(self,data):
        """Get data for the face service: Action Units and Facial Expressions

        Args:
            data ([str]): json of data for the face 
                start:
                time:
                type:


        Returns:
            face_expr_bool(bool): true if a facial expression exists, false if not
            face_expressions (list): array of facial expressions
        """
        face_expr_bool = False
        face_expressions = self._get_facial_expression_data(data)
        [au_bool, aus] = self._get_aus_data(data)
        if au_bool:
            face_expressions.append(aus)
        if face_expressions!=[]:
            face_expr_bool = True
        return face_expr_bool, face_expressions 
