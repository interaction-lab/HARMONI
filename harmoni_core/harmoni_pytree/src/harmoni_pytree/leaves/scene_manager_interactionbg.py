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

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        #self.blackboard_scene.register_key(key=PyTreeNameSpace.interaction.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.interaction.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.interaction.name+"/max_num_scene", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key=PyTreeNameSpace.interaction.name+"/do_trigger", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.analyzer.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.trigger.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_visual= self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key(key="inside", access=py_trees.common.Access.READ)

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

        self.blackboard_scene.interaction.max_num_scene = len(self.context["scene"])
        self.blackboard_scene.interaction.scene_counter = 0
        self.blackboard_scene.utterance = "null"
        self.blackboard_scene.face_exp = "null"
        self.blackboard_scene.therapist_needed = False
        self.blackboard_scene.interaction.do_trigger = "null"

    def initialise(self):
        self.logger.debug("  %s [SceneManagerInteractionBg::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SceneManagerInteractionBg::update()]" % self.name)

        self.blackboard_scene.mainactivity.do_trigger = False
        self.blackboard_scene.therapist_needed = False
        self.blackboard_scene.face_exp = "null"

        if self.blackboard_bot.analyzer.result == "null":
            self.blackboard_scene.interaction.do_trigger = True
            self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.interaction.scene_counter]["utterance"]
            self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.interaction.scene_counter]["face"]
        else:
            if self.blackboard_bot.analyzer.result == "void_answer":
                self.blackboard_scene.therapist_needed = True
                self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
            elif self.blackboard_bot.analyzer.result["intentName"] == "Stop": 
                self.blackboard_scene.therapist_needed = True
                self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
            elif self.blackboard_bot.analyzer.result["intentName"] == "NonHoCapito": 
                self.blackboard_scene.therapist_needed = True
                self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
            elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.FULFILLED.value or self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.READY_FOR_FULFILLMENT.value:
                self.blackboard_scene.utterance = self.blackboard_bot.analyzer.result["message"]
                self.blackboard_scene.interaction.scene_counter += 1
            elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.CONFIRM_INTENT.value:
                self.blackboard_scene.utterance = self.blackboard_bot.analyzer.result["message"]
            elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.FAILED.value:
                self.blackboard_scene.utterance = self.blackboard_bot.analyzer.result["message"]
                self.blackboard_scene.interaction.scene_counter += 1
            elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.ELICIT_SLOT.value:
                #TODO forse da cambiare con ripetere scena corrente
                self.blackboard_scene.utterance = self.blackboard_bot.analyzer.result["message"]
            elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.ELICIT_INTENT.value:
                self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.interaction.scene_counter]["utterance"]
                self.blackboard_scene.interaction.do_trigger = True
            else:
                #qui non dovremmo mai entrare in quanto abbiamo gestito tutti gli stati
                pass
            self.blackboard_bot.analyzer.result = "null"
        
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SceneManagerInteractionBg::terminate()][%s->%s]" % (self.name, self.status, new_status))
