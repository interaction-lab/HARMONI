#!/usr/bin/env python3

# Importing the libraries
from enum import IntEnum, Enum
import yaml
import os

class HelperFunctions:
    def get_child_list(child_name): 
        """Get children from config file"""
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("HARMONI/")
        with open(path[0] + 'HARMONI/harmoni_decision/config/configuration.yaml') as file:
            repos = yaml.load(file, Loader=yaml.FullLoader)
        for repo in repos["repo"]:
            if child_name in repos["repo"][repo]:
                repo_child_name = repo + "_" + child_name
        [child_repo, child_name] = repo_child_name.split("_")
        ids_list = []
        for id_child in repos["repo"][child_repo][child_name]:
            ids_list.append(child_repo + "_" + child_name+ "_" + id_child)
        return ids_list

    def get_child_id(service_name):
        """Get id of the child from service name"""
        service = service_name.split("_")
        id = service[-1]
        return id


class ActionType(IntEnum):
    ON = 1
    OFF = 2
    PAUSE = 3
    REQUEST = 4

class State(IntEnum):
    INIT = 0
    START = 1
    REQUEST = 2
    SUCCESS = 3
    FAILED = 4

class Router(Enum):
    SENSOR = "sensor"
    DIALOGUE = "dialogue"
    ACTUATOR = "actuator"
    DETECTOR = "detector"

class RouterSensor(Enum):
    MICROPHONE = "microphone"
    CAMERA = "camera"
    PATH = "/harmoni/sensing/"

class RouterActuator(Enum):
    SPEAKER = "speaker"
    FACE = "face"
    TTS = "tts"
    WEB = "web"
    PATH = "/harmoni/actuating/"

class RouterDialogue(Enum):
    LEX = "lex"
    PATH = "/harmoni/dialoging/"

class RouterDetector(Enum):
    STT = "stt"
    PATH = "/harmoni/detecting/"