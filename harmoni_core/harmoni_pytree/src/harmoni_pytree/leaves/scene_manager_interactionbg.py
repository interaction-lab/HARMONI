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
        self.blackboard_scene.register_key(key=PyTreeNameSpace.interaction.name+"/do_kid", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_interaction = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.interaction.name)
        self.blackboard_interaction.register_key("inside", access=py_trees.common.Access.WRITE)
        self.blackboard_interaction.register_key("finished", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_interaction.register_key("call_therapist", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.analyzer.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.trigger.name +"/"+"result", access=py_trees.common.Access.WRITE)

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

        self.blackboard_scene.interaction.max_num_scene = len(self.context["scene"])+1
        self.blackboard_scene.interaction.scene_counter = 0
        self.blackboard_scene.utterance = "null"
        self.blackboard_scene.face_exp = "null"
        self.blackboard_interaction.call_therapist = False
        self.blackboard_interaction.inside = False
        self.blackboard_scene.interaction.do_trigger = "null"
        self.blackboard_scene.interaction.do_kid = True

    def initialise(self):
        self.logger.debug("  %s [SceneManagerInteractionBg::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SceneManagerInteractionBg::update()]" % self.name)

        self.blackboard_scene.interaction.do_trigger = False
        self.blackboard_interaction.call_therapist = False
        self.blackboard_interaction.finished = False
        self.blackboard_scene.interaction.do_kid = True
        self.blackboard_interaction.inside = True
        self.blackboard_scene.face_exp = "null"

        print("STATE OF SCENE MANAGER INTERACTION")

        if self.blackboard_scene.interaction.scene_counter == 0:
            print("self.blackboard_scene.interaction.scene_counter == 0")
            self.blackboard_scene.interaction.do_trigger = True
            self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.interaction.scene_counter]["utterance"]
            self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.interaction.scene_counter]["face"]
            self.blackboard_scene.interaction.scene_counter += 1
        else:
            if self.blackboard_bot.analyzer.result == "void_answer":
                print("self.blackboard_bot.analyzer.result == void_answer")
                self.blackboard_interaction.call_therapist = True
                self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
            else:
                intentName = self.blackboard_bot.analyzer.result["intentName"]
                dialogState = self.blackboard_bot.analyzer.result["dialogState"]
                message = self.blackboard_bot.analyzer.result["message"]
                print("Intent name: ", intentName)
                print("Dialog state: ", dialogState)
                if intentName == "Stop": 
                    print("intentName == Stop")
                    self.blackboard_interaction.call_therapist = True
                    self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                    self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
                elif intentName == "NonHoCapito":
                    print("intentName == NonHoCapito") 
                    self.blackboard_interaction.call_therapist = True
                    self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                    self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
                elif dialogState == DialogStateLex.FULFILLED.value or dialogState == DialogStateLex.READY_FOR_FULFILLMENT.value:
                    print("dialogState == FULFILLED")
                    self.blackboard_scene.utterance = message
                    self.blackboard_scene.interaction.do_kid = False
                    self.blackboard_scene.interaction.scene_counter += 1
                elif dialogState == DialogStateLex.CONFIRM_INTENT.value:
                    print("dialogState == CONFIRM_INTENT")
                    self.blackboard_scene.utterance = message
                elif dialogState == DialogStateLex.FAILED.value:
                    print("dialogState == FAILED")
                    self.blackboard_interaction.call_therapist = True
                    self.blackboard_scene.utterance = message
                    
                    self.blackboard_scene.interaction.do_kid = False
                    self.blackboard_scene.interaction.scene_counter += 1
                elif dialogState == DialogStateLex.ELICIT_SLOT.value:
                    print("dialogState == ELICIT_SLOT")
                    self.blackboard_scene.utterance = message
                elif dialogState == DialogStateLex.ELICIT_INTENT.value:
                    print("dialogState == ELICIT_INTENT")
                    self.blackboard_scene.utterance = self.context["scene"][0]["utterance"]
                    self.blackboard_scene.interaction.do_trigger = True
                else:
                    raise
            self.blackboard_bot.analyzer.result = "null"

        self.blackboard_bot.trigger.result =    {
                                                                "message":   self.blackboard_scene.utterance
                                                            }
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SceneManagerInteractionBg::terminate()][%s->%s]" % (self.name, self.status, new_status))

