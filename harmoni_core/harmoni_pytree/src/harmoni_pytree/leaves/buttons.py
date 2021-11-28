#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import rospy
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
        self.max_duration = 4
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
                self.read_serial = []
                self.read_serial = self.wemos.readlines()
                print("@@@@@@@@@@@@@@@@@@")
                print(self.read_serial)
                if len(self.read_serial) != 0:
                    self.read_serial = str(self.read_serial[0])
                    print(self.read_serial)
                    if self.read_serial == "b'pressb2\\r\\n'":
                        print("si")
                        self.blackboard_buttons.result = "si"
                        new_status = py_trees.common.Status.SUCCESS
                    elif self.read_serial == "b'pressb1\\r\\n'":
                        print("no")
                        self.blackboard_buttons.result = "no"
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        print("errore")
                        self.blackboard_buttons.result = "null"
                        new_status = py_trees.common.Status.SUCCESS
                else:
                    new_status = py_trees.common.Status.RUNNING
        self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status


    def terminate(self, new_status):
        self.start_time = None
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

def main():
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    rospy.init_node("buttons_default", log_level=rospy.INFO)

    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=PyTreeNameSpace.buttons.name)
    blackboardProva.register_key("result", access=py_trees.common.Access.READ)
    print(blackboardProva)

    buttons = Buttons("ButtonsPytreeTest")

    buttons.setup()
    try:
        for unused_i in range(0, 50):
            buttons.tick_once()
            time.sleep(1)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass

if __name__ == "__main__":
    main()
