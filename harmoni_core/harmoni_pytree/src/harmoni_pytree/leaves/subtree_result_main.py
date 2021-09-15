#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import random


class SubTreeResultMain(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.blackboard_scene_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name +"/"+ PyTreeNameSpace.mainactivity.name)
        self.blackboard_scene_mainactivity.register_key("max_num_scene", access=py_trees.common.Access.READ) #NEW
        self.blackboard_scene_mainactivity.register_key("scene_counter", access=py_trees.common.Access.WRITE)

        super(SubTreeResultMain, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        self.logger.debug("  %s [SubTreeResultMain::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SubTreeResultMain::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SubTreeResultMain::update()]" % self.name)
        #TODO cancella questa folgia e menti del either or successivo la condizione
        #max_num_scene == scene_counter

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [SubTreeResultMain::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
