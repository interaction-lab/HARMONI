#!/usr/bin/env python3

# Importing the libraries
from enum import IntEnum

class ActionType(IntEnum):
    ON = 1
    OFF = 2
    REQUEST = 3

class State(IntEnum):
    INIT = 0
    END = 1
    START = 2
    STOP = 3
    REQUEST = 4
    RESPONSE = 5
    SUCCESS = 6
    FAILED = 7