#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import random

#TODO prendi spunto da Timer di py_trees
class Timer(py_trees.behaviour.Behaviour):
    def __init__(self,
            variable_namespace: str, 
            variable_name: str,
            name: typing.Union[str, common.Name]=common.Name.AUTO_GENERATED,
    ):
        super(Timer, self).__init__(name)

        self.blackboards = []
        self.blackboard_camera = self.attach_blackboard_client(name=self.name, namespace=variable_namespace)
        self.blackboard_camera.register_key(variable_name, access=py_trees.common.Access.WRITE)

        self.finish_time = None

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def initialise(self):
        """
        Store the expected finishing time.
        """
        if self.finish_time is None:
            self.start_time = time.time()
            self.blackboard_camera.variable_name = 0
        self.feedback_message = "timer started at '{0}' ".format(self.start_time)

        self.logger.debug("  %s [Timer::initialise()]" % self.name)

    def update(self):
        """
        Check current time against the expected finishing time. If it is in excess, flip to
        :data:`~py_trees.common.Status.SUCCESS`.
        """
        self.logger.debug("  %s [Timer::update()]" % self.name)
        elapsed_time = time.time() - self.start_time #time.time() is the current time
            self.feedback_message = "timer ran out [{0}]".format(self.duration)
        self.blackboard_camera.variable_name = elapsed_time
        return common.Status.SUCCESS

    def terminate(self, new_status):
        """
        Clear the expected finishing time.
        """
        self.logger.debug("  %s [Timer::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        if new_status == common.Status.INVALID:
            self.start_time = None