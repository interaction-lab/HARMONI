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
        super(SceneManagerMain, self).__init__(name)

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        #self.blackboard_scene.register_key(PyTreeNameSpace.mainactivity.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.mainactivity.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key=PyTreeNameSpace.mainactivity.name+"/max_num_scene", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key=PyTreeNameSpace.mainactivity.name+"/do_kid", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key=PyTreeNameSpace.mainactivity.name+"/do_trigger", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(key="utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="gesture", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="image", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(key="sound", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.analyzer.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.trigger.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_mainactivity = self.attach_blackboard_client(name=self.name, namespace = PyTreeNameSpace.mainactivity.name)
        self.blackboard_mainactivity.register_key(key="counter_no_answer", access=py_trees.common.Access.WRITE)
        self.blackboard_mainactivity.register_key(key="call_therapist", access=py_trees.common.Access.WRITE)
        self.blackboard_visual= self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key("inside", access=py_trees.common.Access.WRITE)
        self.blackboard_interaction= self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.interaction.name)
        self.blackboard_interaction.register_key("inside", access=py_trees.common.Access.WRITE)

        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self):
        pattern_name = "mainactivity"
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/resources/{pattern_name}.json"
        with open(pattern_script_path, "r") as read_file:
            self.context = json.load(read_file)

        self.counter_non_ho_capito = 0
        self.blackboard_scene.mainactivity.max_num_scene = len(self.context["scene"])
        self.blackboard_mainactivity.counter_no_answer = 0
        self.blackboard_scene.mainactivity.scene_counter = 0
        self.blackboard_scene.utterance = "null"
        self.blackboard_scene.face_exp = "null"
        self.blackboard_scene.gesture = "null"
        self.blackboard_scene.image = "null"
        self.blackboard_scene.sound = "null"
        self.blackboard_mainactivity.call_therapist = False
        self.blackboard_scene.mainactivity.do_kid = False
        self.blackboard_scene.mainactivity.do_trigger = False
        self.error_message = "Mi dispiace, proviamo a chiedere aiuto al terapista"
        self.blackboard_visual.inside = False
        self.blackboard_interaction.inside = False

        self.logger.debug("  %s [SceneManagerMain::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SceneManagerMain::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SceneManagerMain::update()]" % self.name)

        self.blackboard_mainactivity.call_therapist = False
        print("STATE OF SCENE MANAGER MAIN")

        if self.blackboard_visual.inside == True:
            print("visual/inside: true")
            self.blackboard_visual.inside = False
            if not self.blackboard_scene.mainactivity.do_kid:
                self.blackboard_scene.mainactivity.scene_counter -= 1
            self.blackboard_scene.utterance = self.context["error_handling"]["riprendiamo_visual"]["utterance"]
            self.blackboard_scene.face_exp = self.context["error_handling"]["riprendiamo_visual"]["face"]
            self.blackboard_scene.gesture = self.context["error_handling"]["riprendiamo_visual"]["gesture"]
            self.blackboard_scene.image = self.context["error_handling"]["riprendiamo_visual"]["image"]
            self.blackboard_scene.sound = self.context["error_handling"]["riprendiamo_visual"]["sound"]
            self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"]["riprendiamo_visual"]["do_trigger"]=="True"
            self.blackboard_scene.mainactivity.do_kid = self.blackboard_scene.mainactivity.do_trigger 
        elif self.blackboard_interaction.inside == True:
            print("interaction/inside: true")
            self.blackboard_interaction.inside = False
            self.blackboard_mainactivity.counter_no_answer = 0
            self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
            self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
            self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
            self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
            self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
            self.blackboard_scene.mainactivity.do_trigger = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["do_trigger"] == "True"
        else:
            self.blackboard_scene.mainactivity.do_kid = False
            self.blackboard_scene.mainactivity.do_trigger = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["do_trigger"]=="True"
            if self.blackboard_scene.mainactivity.do_trigger == True:
                print("mainactivity/do_trigger = true")
                if self.blackboard_bot.analyzer.result == "null":
                    print("analyzer/result == null")
                    self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                    self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                    self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                    self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                    self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                    self.blackboard_scene.mainactivity.do_kid = True
                    self.counter_non_ho_capito = 0
                else:
                    if self.blackboard_bot.analyzer.result == "void_answer":
                        print("analyzer/result == void_answer")
                        self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                        self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                        self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                        self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                        self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                        self.blackboard_scene.mainactivity.do_kid = True
                    else:
                        dialogState = self.blackboard_bot.analyzer.result["dialogState"]
                        print("dialogState = ", dialogState)
                        if dialogState == DialogStateLex.FULFILLED.value or dialogState == DialogStateLex.READY_FOR_FULFILLMENT.value:
                            intentName = self.blackboard_bot.analyzer.result["intentName"]
                            message = self.blackboard_bot.analyzer.result["message"]
                            print("intentName = ", intentName)
                            print("message = ", message)
                            self.blackboard_scene.utterance = message
                            if intentName == IntentName.STOP.value:
                                self.blackboard_mainactivity.call_therapist = True
                                self.blackboard_scene.face_exp = self.context["error_handling"]["stop"]["face"]
                                self.blackboard_scene.gesture = self.context["error_handling"]["stop"]["gesture"]
                                self.blackboard_scene.image = self.context["error_handling"]["stop"]["image"]
                                self.blackboard_scene.sound = self.context["error_handling"]["stop"]["sound"]
                                self.blackboard_scene.sound = self.context["error_handling"]["stop"]["sound"]
                                self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"]["stop"]["do_trigger"]=="True"
                                self.blackboard_scene.mainactivity.do_kid = self.blackboard_scene.mainactivity.do_trigger
                            elif intentName == IntentName.NOCAPITO.value: 
                                self.blackboard_scene.face_exp = self.context["error_handling"]["no_capito"]["face"]
                                self.blackboard_scene.gesture = self.context["error_handling"]["no_capito"]["gesture"]
                                self.blackboard_scene.image = self.context["error_handling"]["no_capito"]["image"]
                                self.blackboard_scene.sound = self.context["error_handling"]["no_capito"]["sound"]
                                self.blackboard_scene.mainactivity.do_trigger = False
                                self.blackboard_scene.mainactivity.do_kid = False
                            else:
                                if intentName == IntentName.OMBRELLO.value:
                                    self.context["scene"][19]["utterance"] = "Fortunatamente abbiamo portato l'ombrello"
                                    #self.context["scene"][19]["face"] = "[{'start':10, 'type': 'gaze', 'id':'target', 'point': [1,5,10]}]"
                                self.blackboard_scene.mainactivity.scene_counter += 1
                                self.blackboard_scene.face_exp = self.context["error_handling"]["risposta_corretta"]["face"]
                                self.blackboard_scene.gesture = self.context["error_handling"]["risposta_corretta"]["gesture"]
                                self.blackboard_scene.image = self.context["error_handling"]["risposta_corretta"]["image"]
                                self.blackboard_scene.sound = self.context["error_handling"]["risposta_corretta"]["sound"]
                                self.blackboard_scene.mainactivity.do_trigger = False
                                self.blackboard_scene.mainactivity.do_kid = False
                        elif dialogState == DialogStateLex.FAILED.value:
                            intentName = self.blackboard_bot.analyzer.result["intentName"]
                            message = self.blackboard_bot.analyzer.result["message"]
                            print("intentName = ", intentName)
                            print("message = ", message)
                            if intentName == IntentName.OMBRELLO.value:
                                self.context["scene"][18]["utterance"] = "Peccato che non abbiamo portato l'ombrello"
                                #self.context["scene"][18]["face"] = "[{'start': 5, 'type': 'action', 'id': 'happy_face'}]"
                            self.blackboard_scene.mainactivity.scene_counter += 1
                            self.blackboard_scene.utterance = message
                            self.blackboard_scene.face_exp = self.context["error_handling"]["risposta_sbagliata"]["face"]
                            self.blackboard_scene.gesture = self.context["error_handling"]["risposta_sbagliata"]["gesture"]
                            self.blackboard_scene.image = self.context["error_handling"]["risposta_sbagliata"]["image"]
                            self.blackboard_scene.sound = self.context["error_handling"]["risposta_sbagliata"]["sound"]
                            self.blackboard_scene.mainactivity.do_trigger = False
                            self.blackboard_scene.mainactivity.do_kid = False
                        elif dialogState == DialogStateLex.ELICIT_SLOT.value:
                            intentName = self.blackboard_bot.analyzer.result["intentName"]
                            message = self.blackboard_bot.analyzer.result["message"]
                            print("intentName = ", intentName)
                            print("message = ", message)
                            if intentName == IntentName.CARTA.value or intentName == IntentName.PLASTICA.value or intentName == IntentName.VETRO.value:
                                if intentName == IntentName.CARTA.value:
                                    oggetto = "tetrapak"
                                elif intentName == IntentName.PLASTICA.value:
                                    oggetto = "lattina"
                                elif intentName == IntentName.VETRO.value:
                                    oggetto = "bicchiere"
                                self.blackboard_scene.utterance = "confirm " + oggetto
                                self.blackboard_scene.face_exp = self.context["error_handling"]["sei_sicuro"]["face"]
                                self.blackboard_scene.gesture = self.context["error_handling"]["sei_sicuro"]["gesture"]
                                self.blackboard_scene.image = self.context["error_handling"]["sei_sicuro"]["image"]
                                self.blackboard_scene.sound = self.context["error_handling"]["sei_sicuro"]["sound"]
                                self.blackboard_scene.mainactivity.do_trigger = True
                                self.blackboard_scene.mainactivity.do_kid = True
                            else:
                                self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                                self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                                self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                                self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                                self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                                self.blackboard_scene.mainactivity.do_kid = self.blackboard_scene.mainactivity.do_trigger
                        elif dialogState == DialogStateLex.ELICIT_INTENT.value:
                            print("-scene_counter_mainactivity- qui non dovremmo esserci")
                            self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                            self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                            self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                            self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                            self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                            self.blackboard_scene.mainactivity.do_kid = self.blackboard_scene.mainactivity.do_trigger
                        elif dialogState == DialogStateLex.CONFIRM_INTENT.value:
                            print("-scene_counter_mainactivity- qui non dovremmo esserci")
                            self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                            self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                            self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                            self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                            self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                            self.blackboard_scene.mainactivity.do_kid = self.blackboard_scene.mainactivity.do_trigger
            else:
                print("mainactivity/do_trigger = false")
                self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                self.blackboard_mainactivity.counter_no_answer = 0
                self.blackboard_scene.mainactivity.scene_counter += 1

        self.blackboard_bot.trigger.result =    {
                                                    "message": self.blackboard_scene.utterance
                                                }
        self.blackboard_bot.analyzer.result = "null"

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [SceneManagerMain::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

def main():

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
