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
    MICROPHONE = "microphone"
    CAMERA = "camera"

class RouterActuator(Enum):
    SPEAKER = "speaker"
    FACE = "face"
    TTS = "tts"
    WEB = "web"

class RouterDialogue(Enum):
    LEX = "lex"


class RouterDetector(Enum):
    STT = "stt"