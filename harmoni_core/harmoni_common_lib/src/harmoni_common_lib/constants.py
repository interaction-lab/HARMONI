#!/usr/bin/env python3

# Importing the libraries
from enum import IntEnum, Enum


class ActionType(IntEnum):
    OFF = 0
    ON = 1
    PAUSE = 2
    REQUEST = 3
    DO = 4


class State(IntEnum):
    INIT = 0
    START = 1
    REQUEST = 2
    SUCCESS = 3
    FAILED = 4
    PAUSE = 5


class Router(Enum):
    SENSOR = "sensor"
    DIALOGUE = "dialogue"
    ACTUATOR = "actuator"
    DETECTOR = "detector"


class SensorNameSpace(Enum):
    microphone = "/harmoni/sensing/microphone/"
    camera = "/harmoni/sensing/camera/"
    ROUTER = "/harmoni/sensing/"


class ActuatorNameSpace(Enum):
    speaker = "/harmoni/actuating/speaker/"
    face = "/harmoni/actuating/face/"
    tts = "/harmoni/actuating/tts/"
    web = "/harmoni/actuating/web/"
    gesture = "/harmoni/actuating/gesture/"
    ROUTER = "/harmoni/actuating/"


class DialogueNameSpace(Enum):
    bot = "/harmoni/dialoging/bot/"
    ROUTER = "/harmoni/dialoging/"


class DetectorNameSpace(Enum):
    stt = "/harmoni/detecting/stt/"
    face_detect = "/harmoni/detecting/face_detect/"
    ROUTER = "/harmoni/detecting/"


class Resources(Enum):
    face = ["eyes", "mouth"]
