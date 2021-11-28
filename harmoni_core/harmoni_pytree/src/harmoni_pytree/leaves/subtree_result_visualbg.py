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
        #self.blackboard_scene_visual.register_key("max_num_scene", access=py_trees.common.Access.READ) #NEW
        self.blackboard_visual = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key(key="finished", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_visual.register_key(key="call_therapist", access=py_trees.common.Access.READ)
        self.blackboard_face_detect = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.face_detect.name)
        self.blackboard_face_detect.register_key("result", access=py_trees.common.Access.READ)

        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(key="therapist_needed", access=py_trees.common.Access.WRITE)
        

    def setup(self):
        self.blackboard_scene_visual.scene_counter = 0
        self.blackboard_scene.therapist_needed = False
        self.blackboard_visual.finished = True
        self.logger.debug("  %s [SubTreeResultVisualBg::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SubTreeResultVisualBg::initialise()]" % self.name)

    def update(self):
        if self.blackboard_face_detect.result == "null":
            self.blackboard_visual.finished = False
        else:
            self.blackboard_scene_visual.scene_counter = 0
            self.blackboard_visual.finished = True

        if self.blackboard_visual.call_therapist:
            self.blackboard_scene.therapist_needed = True        
            
        self.logger.debug("  %s [SubTreeResultVisualBg::update()]" % self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SubTreeResultVisualBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
