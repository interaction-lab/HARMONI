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
        
        self.name = name

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/max_num_scene", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key=PyTreeNameSpace.visual.name+"/do_trigger", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.analyzer.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.trigger.name +"/"+"result", access=py_trees.common.Access.WRITE)
        #self.blackboard_card_detection = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.card_detect.name)
        #self.blackboard_card_detection.register_key("result", access=py_trees.common.Access.WRITE)
        """
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        """
        self.blackboard_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.mainactivity.name)
        self.blackboard_mainactivity.register_key("counter_no_answer", access=py_trees.common.Access.WRITE)

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

        self.blackboard_scene.visual.max_num_scene = len(self.context["scene"]) #da cambiare
        self.blackboard_mainactivity.counter_no_answer = 0
        self.blackboard_scene.visual.scene_counter = 0 
        self.blackboard_scene.utterance = None
        self.blackboard_scene.face_exp = None
        self.blackboard_scene.therapist_needed = None
        self.blackboard_scene.visual.do_trigger = None
        self.logger.debug("  %s [SceneManagerVisualBg::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SceneManagerVisualBg::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SceneManagerVisualBg::update()]" % self.name)
        self.blackboard_mainactivity.counter_no_answer = 0

        #scena iniziale ovvero la zero, è l'unica in cui va bot_trigger e serve l'utterance. 
        if self.blackboard_scene.visual.scene_counter == 0: 
            self.blackboard_scene.visual.do_trigger = True
            self.blackboard_scene.utterance = self.context["scene"][self.scene_counter]["utterance"]
            self.blackboard_scene.face_exp = self.context["scene"][self.scene_counter]["face"]
            self.blackboard_scene.visual.scene_counter += 1

        else:
            self.blackboard_scene.visual.do_trigger = False #deve essere usato solo bot_analyzer dopo la scena 0
            if self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.FAILED:
                self.blackboard_bot.trigger.result = self.blackboard_bot.analyzer.result["message"]
                self.blackboard_scene.therapist_needed = True
            else:
                if self.blackboard_scene.visual.scene_counter <= 2: 
                    self.blackboard_bot.trigger.result = self.blackboard_bot.analyzer.result["message"]
                    self.blackboard_scene.visual.scene_counter += 1
                else:
                    self.blackboard_bot.trigger.result = self.context["terapista"]["utterance"]
                    self.blackboard_scene.face_exp = self.context["terapista"]["face"]
                    self.blackboard_scene.therapist_needed = True
        
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        if new_status == py_trees.common.Status.INVALID:
            self.scene_counter = 0
        """
        self.logger.debug("  %s [SceneManagerVisualBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
