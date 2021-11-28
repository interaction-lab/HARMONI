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
        #self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.visual.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.visual.name+"/do_trigger", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key="utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_visual = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key(key="inside", access=py_trees.common.Access.WRITE)
        self.blackboard_visual.register_key(key="finished", access=py_trees.common.Access.WRITE)
        self.blackboard_visual.register_key(key="call_therapist", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.analyzer.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.trigger.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_mainactivity = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.mainactivity.name)
        self.blackboard_mainactivity.register_key(key="counter_no_answer", access=py_trees.common.Access.WRITE)
        self.blackboard_interaction = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.interaction.name)
        self.blackboard_interaction.register_key("finished", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_interaction.register_key("inside", access=py_trees.common.Access.WRITE)

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

        #self.blackboard_scene.visual.max_num_scene = len(self.context["scene"]) #da cambiare
        self.blackboard_mainactivity.counter_no_answer = 0
        self.blackboard_scene.visual.scene_counter = 0 
        self.blackboard_scene.utterance = "null"
        self.blackboard_scene.face_exp = "null"
        self.blackboard_visual.call_therapist = False
        self.blackboard_visual.inside = False
        self.blackboard_scene.visual.do_trigger = "null"

        self.logger.debug("  %s [SceneManagerVisualBg::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SceneManagerVisualBg::initialise()]" % self.name)

    def update(self):

        self.logger.debug("  %s [SceneManagerVisualBg::update()]" % self.name)
        #reset of interaction 
        self.blackboard_mainactivity.counter_no_answer = 0
        self.blackboard_interaction.finished = True
        self.blackboard_interaction.inside = False
        self.blackboard_visual.finished = False
        self.blackboard_visual.inside = True

        print("STATE OF SCENE MANAGER VISAUL")

        if self.blackboard_scene.visual.scene_counter == 0:
            print("self.blackboard_scene.visual.scene_counter == 0")
            self.blackboard_scene.visual.do_trigger = True
            self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.visual.scene_counter]["utterance"]
            self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.visual.scene_counter]["face"]
            self.blackboard_scene.visual.scene_counter += 1
        else:
            self.blackboard_scene.visual.do_trigger = False
            if self.blackboard_bot.analyzer.result == "void_answer":
                print("self.blackboard_bot.analyzer.result == void_answer")
                self.blackboard_visual.call_therapist = True
                self.blackboard_bot.trigger.result =    {
                                                            "message":   self.context["terapista"]["utterance"]
                                                        }
                self.blackboard_scene.face_exp = self.context["terapista"]["face"]
            else:
                dialogState = self.blackboard_bot.analyzer.result["dialogState"]
                message = self.blackboard_bot.analyzer.result["message"]
                if dialogState == DialogStateLex.FAILED.value:
                    print("dialogState == FAILED")
                    self.blackboard_bot.trigger.result =    {
                                                                "message":   message
                                                            }

                    self.blackboard_visual.call_therapist = True
                else:
                    if self.blackboard_scene.visual.scene_counter <= 2:
                        print("self.blackboard_scene.visual.scene_counter <= 2") 
                        self.blackboard_bot.trigger.result =    {
                                                                    "message":   message
                                                                }
                        self.blackboard_scene.visual.scene_counter += 1
                    else:
                        print("self.blackboard_scene.visual.scene_counter > 2") 
                        self.blackboard_bot.trigger.result =    {
                                                                    "message":  self.context["terapista"]["utterance"]
                                                                }
                        self.blackboard_scene.face_exp = self.context["terapista"]["face"]
                        self.blackboard_visual.call_therapist = True
        
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        if new_status == py_trees.common.Status.INVALID:
            self.scene_counter = 0
        """
        self.logger.debug("  %s [SceneManagerVisualBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
