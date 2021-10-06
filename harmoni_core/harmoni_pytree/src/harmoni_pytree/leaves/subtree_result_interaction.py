#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random


class SubTreeResultInteractionBg(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SubTreeResultInteractionBg, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        self.name = name
        self.blackboards = []
        
        self.blackboard_scene_interaction = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name +"/"+ PyTreeNameSpace.interaction.name)
        self.blackboard_scene_interaction.register_key("scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene_interaction.register_key("max_num_scene", access=py_trees.common.Access.READ) #NEW
        self.blackboard_mainactivity = self.attach_blackboard_client(name=self.name, namespace= PyTreeNameSpace.mainactivity.name)
        self.blackboard_mainactivity.register_key("counter_no_answer", access=py_trees.common.Access.WRITE)
        self.blackboard_interaction = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.interaction.name)
        self.blackboard_interaction.register_key("finished", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_interaction.register_key(key="call_therapist", access=py_trees.common.Access.READ)

        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key="therapist_needed", access=py_trees.common.Access.WRITE)
        

    def setup(self):
        self.blackboard_scene.therapist_needed = False
        self.blackboard_mainactivity.counter_no_answer = 0
        self.blackboard_scene_interaction.scene_counter = 0
        self.blackboard_interaction.finished = True

        self.logger.debug("  %s [SubTreeResultInteractionBg::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SubTreeResultInteractionBg::initialise()]" % self.name)

    def update(self):

        if self.blackboard_scene_interaction.scene_counter == self.blackboard_scene_interaction.max_num_scene:
            self.blackboard_scene_interaction.scene_counter = 0
            self.blackboard_interaction.finished = True
        if self.blackboard_interaction.call_therapist:
            self.blackboard_scene.therapist_needed = True
        self.logger.debug("  %s [SubTreeResultInteractionBg::update()]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SubTreeResultInteractionBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
