#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random


class SubTreeResultVisualBg(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SubTreeResultVisualBg, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

        self.name = name
        self.blackboards = []
        self.blackboard_scene_visual = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name + "/" + PyTreeNameSpace.visual.name)
        self.blackboard_scene_visual.register_key("scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene_visual.register_key("max_num_scene", access=py_trees.common.Access.READ) #NEW
        self.blackboard_visual = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key("inside", access=py_trees.common.Access.WRITE)
        self.blackboard_visual.register_key("finished", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_face_detect = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.face_detect.name)
        self.blackboard_face_detect.register_key("result", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [SubTreeResultVisualBg::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SubTreeResultVisualBg::initialise()]" % self.name)

    def update(self):
        #se si è entrati almeno una volta in visual_bg
        if self.blackboard_scene_visual.scene_counter != 0 and self.blackboard_face_detect.result == "null":
            self.blackboard_visual.inside = True
        #caso in cui si è arrivato al numero massimo di scene o il bimbo c'è -->
        if self.blackboard_face_detect.result is not "null":
            self.blackboard_scene_visual.scene_counter = 0
        # if le scene sono più di 2 -->
        # chiama terapista
        # if la frase che ci ha dato il bot ci fa capire che dobbimao chiamare il terapita -->
        # chiama terapista
        self.blackboard_visual.finished = True
        self.logger.debug("  %s [SubTreeResultVisualBg::update()]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SubTreeResultVisualBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
