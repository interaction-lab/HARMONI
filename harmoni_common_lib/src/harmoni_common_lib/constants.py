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
    #DETECTOR = "detector"

class RouterSensor(IntEnum):
    pc_microphone = 1
    pc_camera = 2

class RouterActuator(IntEnum):
    pc_speaker = 1
    pc_face = 2
    tts = 3
    web = 4

class RouterDialogue(IntEnum):
    lex = 1


class RouterDetector(IntEnum):
    stt = 1