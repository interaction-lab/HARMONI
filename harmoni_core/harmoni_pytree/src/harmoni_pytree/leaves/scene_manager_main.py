#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random
import os
import json
import time
import rospkg

class SceneManagerMain(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super(SceneManagerMain, self).__init__(name)

        self.scene_counter = 0

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        #self.blackboard_scene.register_key(PyTreeNameSpace.mainactivity.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.mainactivity.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.mainactivity.name+"/max_num_scene", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key=PyTreeNameSpace.mainactivity.name+"/do_dialogue", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key="utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="gesture", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="image", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="sound", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key(key="result", access=py_trees.common.Access.WRITE)
        self.blackboard_invalid_mainactivity = self.attach_blackboard_client(name=self.name, namespace = PyTreeNameSpace.invalid_response.name+"/"+PyTreeNameSpace.mainactivity.name)
        self.blackboard_invalid_mainactivity.register_key(key="counter_no_answer", access=py_trees.common.Access.WRITE)
        """
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        """
        self.blackboard_visual= self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key("inside", access=py_trees.common.Access.WRITE)
        self.blackboard_interaction= self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.interaction.name)
        self.blackboard_interaction.register_key("inside", access=py_trees.common.Access.WRITE)
        """
        self.blackboard_card_detect = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.card_detect.name)
        self.blackboard_card_detect.register_key("result", access=py_trees.common.Access.READ)
        """

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        pattern_name = "mainactivity"
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/resources/{pattern_name}.json"
        with open(pattern_script_path, "r") as read_file:
          self.context = json.load(read_file)

        self.scene_counter = 0
        self.counter_non_ho_capito = 0
        self.blackboard_scene.mainactivity.max_num_scene = len(self.context["scene"])
        self.blackboard_invalid_mainactivity.counter_no_answer = 0
        self.blackboard_scene.mainactivity.scene_counter = self.scene_counter 
        self.blackboard_scene.utterance = None
        self.blackboard_scene.face_exp = None
        self.blackboard_scene.gesture = None
        self.blackboard_scene.image = None
        self.blackboard_scene.sound = None
        self.blackboard_scene.therapist_needed = None
        self.blackboard_scene.mainactivity.do_dialogue = None

        self.logger.debug("  %s [SceneManagerMain::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SceneManagerMain::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SceneManagerMain::update()]" % self.name)
        if self.blackboard_visual.inside == True:
            self.blackboard_visual.inside = False
            self.scene_counter -= 1
            self.blackboard_scene.utterance = self.context["error_handling"]["riprendiamo"]["utterance"]
            self.blackboard_scene.face_exp = self.context["error_handling"]["riprendiamo"]["face"]
            self.blackboard_scene.gesture = self.context["error_handling"]["riprendiamo"]["gesture"]
            self.blackboard_scene.image = self.context["error_handling"]["riprendiamo"]["image"]
            self.blackboard_scene.sound = self.context["error_handling"]["riprendiamo"]["sound"]
            self.blackboard_scene.mainactivity.do_dialogue = self.context["error_handling"]["riprendiamo"]["do_dialogue"]
        elif self.blackboard_interaction.inside == True:
            self.blackboard_interaction.inside = False
            self.scene_counter -= 1
            self.blackboard_scene.utterance = self.context["error_handling"]["riprendiamo"]["utterance"]
            self.blackboard_scene.face_exp = self.context["error_handling"]["riprendiamo"]["face"]
            self.blackboard_scene.gesture = self.context["error_handling"]["riprendiamo"]["gesture"]
            self.blackboard_scene.image = self.context["error_handling"]["riprendiamo"]["image"]
            self.blackboard_scene.sound = self.context["error_handling"]["riprendiamo"]["sound"]
            self.blackboard_scene.mainactivity.do_dialogue = self.context["error_handling"]["riprendiamo"]["do_dialogue"]
        elif self.blackboard_scene.mainactivity.do_dialogue == True:
            if self.blackboard_bot.result.split('-')[1] == DialogStateLex.FULFILLED: #1 prende lo stato dell'intent
                self.scene_counter += 1
                self.counter_non_ho_capito = 0
                self.blackboard_scene.utterance = self.context["scene"][self.scene_counter]["utterance"]
                self.blackboard_scene.face_exp = self.context["scene"][self.scene_counter]["face"]
                self.blackboard_scene.gesture = self.context["scene"][self.scene_counter]["gesture"]
                self.blackboard_scene.image = self.context["scene"][self.scene_counter]["image"]
                self.blackboard_scene.sound = self.context["scene"][self.scene_counter]["sound"]
                self.blackboard_scene.mainactivity.do_dialogue = self.context["scene"][self.scene_counter]["do_dialogue"]
            elif self.blackboard_bot.result.split('-')[2] == "Stop": #2 prende il nome dell'intent
                self.blackboard_scene.therapist_needed = True
                self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
                self.blackboard_scene.gesture = self.context["error_handling"]["terapista"]["gesture"]
                self.blackboard_scene.image = self.context["error_handling"]["terapista"]["image"]
                self.blackboard_scene.sound = self.context["error_handling"]["terapista"]["sound"]
                self.blackboard_scene.mainactivity.do_dialogue = self.context["scene"]["terapista"]["do_dialogue"]
            elif self.blackboard_bot.result.split('-')[2] == "NonHoCapito": #0 prenderebbe il messaggio di risposta di lex
                self.counter_non_ho_capito += 1
                if self.counter_non_ho_capito == 2:
                    self.blackboard_scene.therapist_needed = True
                    self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                    self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
                    self.blackboard_scene.gesture = self.context["error_handling"]["terapista"]["gesture"]
                    self.blackboard_scene.image = self.context["error_handling"]["terapista"]["image"]
                    self.blackboard_scene.sound = self.context["error_handling"]["terapista"]["sound"]
                    self.blackboard_scene.mainactivity.do_dialogue = self.context["scene"]["terapista"]["do_dialogue"]
                    self.counter_non_ho_capito = 0
                else:
                    self.blackboard_scene.utterance = self.context["scene"][self.scene_counter]["utterance"]
                    self.blackboard_scene.face_exp = self.context["scene"][self.scene_counter]["face"]
                    self.blackboard_scene.gesture = self.context["scene"][self.scene_counter]["gesture"]
                    self.blackboard_scene.image = self.context["scene"][self.scene_counter]["image"]
                    self.blackboard_scene.sound = self.context["scene"][self.scene_counter]["sound"]
                    self.blackboard_scene.mainactivity.do_dialogue = self.context["scene"][self.scene_counter]["do_dialogue"]
        else:
          self.blackboard_scene.utterance = self.context["scene"][self.scene_counter]["utterance"]
          self.blackboard_bot.result = self.blackboard_scene.utterance
          self.blackboard_scene.face_exp = self.context["scene"][self.scene_counter]["face"]
          self.blackboard_scene.gesture = self.context["scene"][self.scene_counter]["gesture"]
          self.blackboard_scene.image = self.context["scene"][self.scene_counter]["image"]
          self.blackboard_scene.sound = self.context["scene"][self.scene_counter]["sound"]
          self.blackboard_scene.mainactivity.do_dialogue = self.context["scene"][self.scene_counter]["do_dialogue"]
          self.scene_counter += 1

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SceneManagerMain::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

def main():
    """
    Entry point for the demo script.
    """

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    action = SceneManagerMain("ppp")
    action.setup()
    try:
        for unused_i in range(0, 12):
            action.tick_once()
            time.sleep(0.5)
        print("\n")
    except KeyboardInterrupt:
        pass
if __name__ == "__main__":
  main()