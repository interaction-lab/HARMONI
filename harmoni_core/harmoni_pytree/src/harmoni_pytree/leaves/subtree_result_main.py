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
        self.blackboard_scene_mainactivity.register_key("scene_counter", access=py_trees.common.Access.WRITE)        

    def setup(self):
        self.logger.debug("  %s [SubTreeResultMain::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SubTreeResultMain::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SubTreeResultMain::update()]" % self.name)
        #TODO cancella questa folgia e menti del either or successivo la condizione
        #max_num_scene == scene_counter
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SubTreeResultMain::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
