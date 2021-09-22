#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random


class SubTreeResultInteractionBg(py_trees.behaviour.Behaviour):
    def __init__(self, name):


        self.name = name
        self.blackboards = []
        
        self.blackboard_scene_interaction = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name +"/"+ PyTreeNameSpace.interaction.name)
        self.blackboard_scene_interaction.register_key("scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene_interaction.register_key("max_num_scene", access=py_trees.common.Access.READ) #NEW
        self.blackboard_invalid_response_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.invalid_response.name +"/"+ PyTreeNameSpace.mainactivity.name)
        self.blackboard_invalid_response_mainactivity.register_key("counter_no_answer", access=py_trees.common.Access.WRITE)
        self.blackboard_interaction = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.interaction.name)
        self.blackboard_interaction.register_key("inside", access=py_trees.common.Access.WRITE)
        
        super(SubTreeResultInteractionBg, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        self.logger.debug("  %s [SubTreeResultInteractionBg::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SubTreeResultInteractionBg::initialise()]" % self.name)

    def update(self):
        
        #se si è entrati almeno una volta in interaction_bg
        if self.blackboard_invalid_response_mainactivity.counter_no_answer >= 2:
          self.blackboard_interaction.inside = True
        #caso in cui si è arrivato al numero massimo di scene -->
        if self.blackboard_scene_interaction.scene_counter == self.blackboard_scene_interaction.max_num_scene:
          self.blackboard_invalid_response_mainactivity.counter_no_answer = 0
          self.blackboard_scene_interaction.scene_counter = 0

        self.logger.debug("  %s [SubTreeResultInteractionBg::update()]" % self.name)

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [SubTreeResultInteractionBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
