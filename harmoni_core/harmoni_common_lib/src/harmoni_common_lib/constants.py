#!/usr/bin/env python3

# Importing the libraries
from enum import IntEnum, Enum


class ActionType(IntEnum):
    """Specifies all valid action types.
    
    """
    OFF = 0
    ON = 1
    PAUSE = 2
    REQUEST = 3
    DO = 4


class State(IntEnum):
    """State describes the status of a service.

    INIT: The service has been initialized and is ready to recieve a request.
    START: The service is doing a long running action.
    REQUEST: The service has made a request and is waiting for the response.
    SUCCESS: The service has successfully completed an action or request.
    FAILED: The service failed to complete an action or request.
    PAUSE: The service is paused.
    """
    INIT = 0
    START = 1
    REQUEST = 2
    SUCCESS = 3
    FAILED = 4
    PAUSE = 5


class SensorNameSpace(Enum):
    microphone = "/harmoni/sensing/microphone/"
    camera = "/harmoni/sensing/camera/"


class ActuatorNameSpace(Enum):
    speaker = "/harmoni/actuating/speaker/"
    face = "/harmoni/actuating/face/"
    tts = "/harmoni/actuating/tts/"
    web = "/harmoni/actuating/web/"
    gesture = "/harmoni/actuating/gesture/"


class DialogueNameSpace(Enum):
    bot = "/harmoni/dialoging/bot/"


class DetectorNameSpace(Enum):
    stt = "/harmoni/detecting/stt/"
    face_detect = "/harmoni/detecting/face_detect/"
    card_detect = "/harmoni/detecting/card_detect/"
    imageai_yolo = "/harmoni/detecting/imageai/yolo"
    imageai_custom_yolo = "/harmoni/detecting/imageai/custom_yolo"



class Resources(Enum):
    face = ["eyes", "mouth", "nose"]


class PyTreeNameSpace(Enum):
    scene = "scene"
    visual = "visual"
    interaction = "interaction"
    mainactivity = "mainactivity"
    timer = "timer"
    invalid_response = "invalid_response"


