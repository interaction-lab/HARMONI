#!/usr/bin/env python3

# Importing the libraries
from enum import IntEnum, Enum

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
    microphone = "/harmoni/sensing/microphone/"
    camera = "/harmoni/sensing/camera/"
    ROUTER = "/harmoni/sensing/"

class RouterActuator(Enum):
    speaker = "/harmoni/actuating/speaker/"
    face = "/harmoni/actuating/face/"
    tts = "/harmoni/actuating/tts/"
    web = "/harmoni/actuating/web/"
    ROUTER = "/harmoni/actuating/"

class RouterDialogue(Enum):
    lex = "/harmoni/dialoging/lex/"
    ROUTER = "/harmoni/dialoging/"

class RouterDetector(Enum):
    stt = "/harmoni/detecting/stt/"
    face_detect = "/harmoni/detecting/face_detect/"
    ROUTER = "/harmoni/detecting/"