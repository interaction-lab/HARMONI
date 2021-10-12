#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random
import serial
import time

class Buttons(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name
        self.blackboards = []
        self.blackboard_buttons = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.buttons.name)
        self.blackboard_buttons.register_key("result", access=py_trees.common.Access.WRITE)

        super(Buttons, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        self.wemos = serial.Serial("/dev/ttyUSB0",timeout=1)
        self.blackboard_buttons.result = "null"
        self.read_serial = []
        self.start_time = None
        self.max_duration = 6
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        self.blackboard_buttons.result = "null"
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        if self.start_time == None:
            self.start_time = time.time() #time.time() is the current time
            print("Timer started at: ", self.start_time)
            new_status = py_trees.common.Status.RUNNING
        else:
            self.elapsed_time = time.time() - self.start_time
            print("elapsed buttons: ",self.elapsed_time)
            if self.max_duration < self.elapsed_time:
                print("Timeout buttons!")
                self.start_time = None
                self.blackboard_buttons.result = "null"
                new_status = py_trees.common.Status.SUCCESS
            else:
                self.read_serial = self.wemos.readlines()
                print("@@@@@@@@@@@@@@@@@@")
                print(self.read_serial)
                if len(self.read_serial) != 0:
                    if self.read_serial[0] == "b'pressb2\r\n":
                        self.blackboard_buttons.result = "si"
                        new_status = py_trees.common.Status.SUCCESS
                    elif self.read_serial[0] == "b'pressb1\r\n":
                        self.blackboard_buttons.result = "no"
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        self.blackboard_buttons.result = "null"
                        new_status = py_trees.common.Status.SUCCESS
                else:
                    new_status = py_trees.common.Status.RUNNING
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status


    def terminate(self, new_status):
        self.start_time = None
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
