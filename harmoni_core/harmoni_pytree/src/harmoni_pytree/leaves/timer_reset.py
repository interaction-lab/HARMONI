#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import time
import typing
import random

#TODO prendi spunto da TimerReset di py_trees
class TimerReset(py_trees.behaviour.Behaviour):
    def __init__(self,
            variable_name: str,
            name: typing.Union[str, py_trees.common.Name]=py_trees.common.Name.AUTO_GENERATED,
    ):
        super(TimerReset, self).__init__(name)

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
        self.blackboard.register_key(key=self.duration_key, access=py_trees.common.Access.READ)

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("  %s [TimerReset::initialise()]" % self.name)

    def update(self):
        timer = self.blackboard.get(self.timer_key)
        duration = self.blackboard.get(self.duration_key)
        if timer == duration:
            self.blackboard.set(
                self.duration_key,
                0,
                overwrite=True
            )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [TimerReset::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))