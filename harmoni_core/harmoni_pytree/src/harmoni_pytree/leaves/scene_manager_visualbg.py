#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import py_trees
import rospkg
import random

from harmoni_common_lib.constants import *

class SceneManagerVisualBg(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        self.name = name
        self.scene_counter = 0

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/max_num_scene", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.READ)
        """
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        """
        self.blackboard_invalid_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.invalid_response.name +"/"+ PyTreeNameSpace.mainactivity.name)
        self.blackboard_invalid_mainactivity.register_key("counter_no_answer", access=py_trees.common.Access.WRITE)

        super(SceneManagerVisualBg, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):

        #this is the name of the json without the extension
        json_name = "visual_bg"
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/resources/{json_name}.json"
        with open(pattern_script_path, "r") as read_file:
          self.context = json.load(read_file)

        self.blackboard_scene.visual.max_num_scene = len(self.context["scene"])
        self.blackboard_invalid_mainactivity.counter_no_answer = 0
        self.blackboard_scene.visual.scene_counter = self.scene_counter 
        self.blackboard_scene.utterance = None
        self.blackboard_scene.face_exp = None
        self.blackboard_scene.therapist_needed = None

        self.logger.debug("  %s [SceneManagerVisualBg::setup()]" % self.name)

    def initialise(self):

        self.logger.debug("  %s [SceneManagerVisualBg::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SceneManagerVisualBg::update()]" % self.name)
        self.scene_counter += 1
        self.blackboard_invalid_mainactivity.counter_no_answer = 0
        self.blackboard_scene.visual.scene_counter = self.scene_counter 
        """
        if intent raggiunto:
          self.scene_counter += 1
          setta tutte le bb con quello che sta dentro context
        elif:
          è la seconda volta che fai questo BG --> 
            self.blackboard_scene.therapist_needed = True
            fai partire intent terapista
        elif:
          scene_counter == 0 -->
          setta tutte le bb con quello che sta dentro context
        """
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID:
            self.scene_counter = 0
        self.logger.debug("  %s [SceneManagerVisualBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
