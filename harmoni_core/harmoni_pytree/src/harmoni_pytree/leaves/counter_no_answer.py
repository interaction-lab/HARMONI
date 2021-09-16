#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import random
import typing
from py_trees import common

#TODO prendi spunto da Count di py_trees
class CounterNoAnswer(py_trees.behaviour.Behaviour):
    def __init__(self,
            variable_name: str,
            name: typing.Union[str, common.Name]=common.Name.AUTO_GENERATED,
    ):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        self.count_no_answer = 0

        self.blackboards = []
        self.blackboard_camera = self.attach_blackboard_client(name=self.name)
        self.blackboard_camera.register_key(variable_name, access=py_trees.common.Access.WRITE)

        super(CounterNoAnswer, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):
        self.logger.debug("  %s [CounterNoAnswer::initialise()]" % self.name)

    def update(self):
        self.count_no_answer += 1
        self.blackboard_camera.variable_name = self.count_no_answer

        self.logger.debug("  %s [CounterNoAnswer::update()][# = %s]" % (self.name,self.count_no_answer))

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        if new_status == common.Status.INVALID:
            self.count_no_answer = 0
            self.blackboard_camera.variable_name = self.count_no_answer
        self.feedback_message = ""

        self.logger.debug("  %s [CounterNoAnswer::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
