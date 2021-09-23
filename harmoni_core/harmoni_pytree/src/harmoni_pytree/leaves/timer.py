#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import time
import typing
import random

class Timer(py_trees.behaviour.Behaviour):
    def __init__(self,
            variable_name: str,
            duration = 0,
            name: typing.Union[str, py_trees.common.Name]=py_trees.common.Name.AUTO_GENERATED,
    ):
        super(Timer, self).__init__(name)

        self.blackboards = []
        self.timer_key = variable_name
        name_components = variable_name.split('/')

        self.duration_key = ""
        self.namespace = name_components[0:len(name_components)-1]
        for element in self.namespace:
            self.duration_key += str(element)+"/"
        self.duration_key += "duration"

        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key=self.timer_key, access=py_trees.common.Access.WRITE)
        self.blackboard.set(
            self.timer_key,
            0,
            overwrite=True
        )
        self.blackboard.register_key(key=self.duration_key, access=py_trees.common.Access.WRITE)
        self.blackboard.set(
            self.duration_key,
            duration,
            overwrite=True
        )
        self.start_time = None
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("  %s [Timer::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Timer::update()]" % self.name)
        timer = self.blackboard.get(self.timer_key)
        if timer == 0:
           self.start_time = time.time() #time.time() is the current time
           self.feedback_message = "timer started at '{0}' ".format(self.start_time)
           elapsed_time = time.time() - self.start_time
        else:
            elapsed_time = time.time() - self.start_time #time.time() is the current time
            self.feedback_message = "timer running from [{0}]".format(self.elapsed_time)
        self.blackboard.set(
            self.timer_key,
            elapsed_time,
            overwrite=True
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [Timer::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        if new_status == py_trees.common.Status.INVALID:
            self.start_time = None
            self.blackboard.set(
                self.timer_key,
                0,
                overwrite=True
            )