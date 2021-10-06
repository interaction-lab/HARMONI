#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random


class SubTreeResultMain(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SubTreeResultMain, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard_scene_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name +"/"+ PyTreeNameSpace.mainactivity.name)
        self.blackboard_scene_mainactivity.register_key("max_num_scene", access=py_trees.common.Access.READ) #NEW
        self.blackboard_scene_mainactivity.register_key("scene_counter", access=py_trees.common.Access.READ)
        self.blackboard_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.mainactivity.name)
        self.blackboard_mainactivity.register_key(key="finished", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_mainactivity.register_key(key="call_therapist", access=py_trees.common.Access.READ)
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key="therapist_needed", access=py_trees.common.Access.WRITE)
        

    def setup(self):
        self.blackboard_scene.therapist_needed = False
        self.blackboard_mainactivity.finished = False
        self.logger.debug("  %s [SubTreeResultMain::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SubTreeResultMain::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SubTreeResultMain::update()]" % self.name)
        if self.blackboard_scene_mainactivity.max_num_scene == self.blackboard_scene_mainactivity.scene_counter:
            self.blackboard_mainactivity.finished = True
        if self.blackboard_mainactivity.call_therapist:
            self.blackboard_scene.therapist_needed = True
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SubTreeResultMain::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
