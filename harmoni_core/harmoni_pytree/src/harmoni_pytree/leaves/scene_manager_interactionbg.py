#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random
import rospkg
import json
import os


class SceneManagerInteractionBg(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        self.name = name
        self.scene_counter = 0

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(PyTreeNameSpace.interaction.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.interaction.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.interaction.name+"/max_num_scene", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key=PyTreeNameSpace.interaction.name+"/do_trigger", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.analyzer.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.trigger.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_visual= self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key("inside", access=py_trees.common.Access.READ)

        self.blackboard_invalid_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.invalid_response.name +"/"+ PyTreeNameSpace.mainactivity.name)
        self.blackboard_invalid_mainactivity.register_key("counter_no_answer", access=py_trees.common.Access.WRITE)
        
        super(SceneManagerInteractionBg, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))


    def setup(self):
        self.logger.debug("  %s [SceneManagerInteractionBg::setup()]" % self.name)
        #this is the name of the json without the extension
        json_name = "interaction_bg"
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/resources/{json_name}.json"
        with open(pattern_script_path, "r") as read_file:
          self.context = json.load(read_file)

        self.scene_counter = 0
        self.blackboard_scene.interaction.max_num_scene = len(self.context["scene"])
        self.blackboard_invalid_mainactivity.counter_no_answer = 0
        self.blackboard_scene.interaction.scene_counter = self.scene_counter 
        self.blackboard_scene.utterance = None
        self.blackboard_scene.face_exp = None
        self.blackboard_scene.therapist_needed = None
        self.blackboard_scene.interaction.do_trigger = None

    def initialise(self):
        self.logger.debug("  %s [SceneManagerInteractionBg::initialise()]" % self.name)

    def update(self):

      #TODO settare variabile counter_no_answer

      #scena iniziale ovvero la zero, è l'unica in cui va bot_trigger e serve l'utterance. 
      if self.scene_counter == 0: 
          self.blackboard_scene.interaction.do_trigger = True
          self.blackboard_scene.utterance = self.context["scene"][self.scene_counter]["utterance"]
          #oppure
          #self.blackboard_scene.utterance = "il bimbo non c'è"
          self.blackboard_scene.face_exp = "null"
          self.scene_counter += 1
      else:
          self.blackboard_scene.interaction.do_trigger = False #deve essere usato solo bot_analyzer dopo la scena 0
          if self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.FULFILLED :
            self.blackboard_bot.trigger.result = self.blackboard_bot.analyzer.result["message"]
            #raggiunte scene massime, fine. Incrementiamo scene?
            self.blackboard_scene.interaction.scene_counter = self.blackboard_scene.interaction.max_num_scene

          elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.CONFIRM_INTENT :
            self.blackboard_bot.trigger.result = self.blackboard_bot.analyzer.result["message"]

          elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.FAILED:
            self.blackboard_bot.trigger.result = self.blackboard_bot.analyzer.result["message"]
            self.blackboard_scene.therapist_needed = True
            #self.blackboard_scene.visual.scene_counter = self.blackboard_scene.visual.max_num_scene

        self.logger.debug("  %s [SceneManagerInteractionBg::update()]" % self.name)
     
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SceneManagerInteractionBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
