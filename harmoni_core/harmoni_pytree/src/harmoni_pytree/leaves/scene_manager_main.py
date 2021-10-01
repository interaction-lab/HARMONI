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

        self.do_trigger_old = False

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
        self.blackboard_scene.register_key(key="therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.analyzer.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_bot.register_key(key=PyTreeNameSpace.trigger.name +"/"+"result", access=py_trees.common.Access.WRITE)
        self.blackboard_mainactivity = self.attach_blackboard_client(name=self.name, namespace = PyTreeNameSpace.mainactivity.name)
        self.blackboard_mainactivity.register_key(key="counter_no_answer", access=py_trees.common.Access.WRITE)
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
        self.blackboard_scene.therapist_needed = False
        self.blackboard_scene.mainactivity.do_kid = "null"
        self.blackboard_scene.mainactivity.do_trigger = "null"
        self.blackboard_mainactivity.counter_no_answer = 0

        self.blackboard_visual.inside = False
        self.blackboard_interaction.inside = False

        self.logger.debug("  %s [SceneManagerMain::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [SceneManagerMain::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [SceneManagerMain::update()]" % self.name)

        self.do_trigger_old = self.blackboard_scene.mainactivity.do_trigger
        self.blackboard_scene.mainactivity.do_trigger = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["do_trigger"]=="True"
        self.blackboard_scene.mainactivity.do_kid = False
        self.blackboard_scene.therapist_needed = False

        print("STATE OF SCENE MANAGER MAIN")

        if self.blackboard_visual.inside == True:
            print("self.blackboard_visual.inside")
            self.blackboard_visual.inside = False
            if not self.do_trigger_old:
                self.blackboard_scene.mainactivity.scene_counter -= 1
            self.blackboard_scene.utterance = self.context["error_handling"]["riprendiamo_visual"]["utterance"]
            self.blackboard_scene.face_exp = self.context["error_handling"]["riprendiamo_visual"]["face"]
            self.blackboard_scene.gesture = self.context["error_handling"]["riprendiamo_visual"]["gesture"]
            self.blackboard_scene.image = self.context["error_handling"]["riprendiamo_visual"]["image"]
            self.blackboard_scene.sound = self.context["error_handling"]["riprendiamo_visual"]["sound"]
            self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"]["riprendiamo_visual"]["do_trigger"]=="True"
        elif self.blackboard_interaction.inside == True:
            print("self.blackboard_interaction.inside")
            self.blackboard_interaction.inside = False
            if not self.do_trigger_old:
                self.blackboard_scene.mainactivity.scene_counter -= 1
            self.blackboard_scene.utterance = self.context["error_handling"]["riprendiamo_interacion"]["utterance"]
            self.blackboard_scene.face_exp = self.context["error_handling"]["riprendiamo_interacion"]["face"]
            self.blackboard_scene.gesture = self.context["error_handling"]["riprendiamo_interacion"]["gesture"]
            self.blackboard_scene.image = self.context["error_handling"]["riprendiamo_interacion"]["image"]
            self.blackboard_scene.sound = self.context["error_handling"]["riprendiamo_interacion"]["sound"]
            self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"]["riprendiamo_interacion"]["do_trigger"]=="True"
        elif self.blackboard_scene.mainactivity.do_trigger == True:
            print("self.blackboard_scene.mainactivity.do_trigger == True")
            if self.blackboard_bot.analyzer.result == "null":
                print("self.blackboard_bot.analyzer.result == null")
                self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                self.blackboard_scene.mainactivity.do_kid = True
                self.counter_non_ho_capito = 0
            else:
                if self.blackboard_bot.analyzer.result == "void_answer":
                    print("self.blackboard_bot.analyzer.result == void_answer")
                    #ripeti l'ultima scena
                    self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                    self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                    self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                    self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                    self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                    self.blackboard_scene.mainactivity.do_kid = True
                else:
                    print("Intent name: ",self.blackboard_bot.analyzer.result["intentName"])
                    print("Dialog state: ",self.blackboard_bot.analyzer.result["dialogState"])
                    if self.blackboard_bot.analyzer.result["intentName"] == "Stop":
                        print("intentName == Stop")
                        self.blackboard_scene.therapist_needed = True
                        self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                        self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
                        self.blackboard_scene.gesture = self.context["error_handling"]["terapista"]["gesture"]
                        self.blackboard_scene.image = self.context["error_handling"]["terapista"]["image"]
                        self.blackboard_scene.sound = self.context["error_handling"]["terapista"]["sound"]
                        self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"]["terapista"]["do_trigger"]=="True"
                    elif self.blackboard_bot.analyzer.result["intentName"] == "NonHoCapito":
                        print("intentName == NonHoCapito")
                        self.counter_non_ho_capito += 1
                        if self.counter_non_ho_capito < 2:
                            self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                            self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                            self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                            self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                            self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                            self.blackboard_scene.mainactivity.do_kid = True
                        else:
                            self.blackboard_scene.therapist_needed = True
                            self.blackboard_scene.utterance = self.context["error_handling"]["terapista"]["utterance"]
                            self.blackboard_scene.face_exp = self.context["error_handling"]["terapista"]["face"]
                            self.blackboard_scene.gesture = self.context["error_handling"]["terapista"]["gesture"]
                            self.blackboard_scene.image = self.context["error_handling"]["terapista"]["image"]
                            self.blackboard_scene.sound = self.context["error_handling"]["terapista"]["sound"]
                            self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"]["terapista"]["do_trigger"]=="True"
                    elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.FULFILLED.value or self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.READY_FOR_FULFILLMENT.value:
                        print("dialogState == FULFILLED")
                        self.blackboard_scene.mainactivity.scene_counter += 1
                        #TODO prendi la risposta di lex e non questa
                        self.blackboard_scene.utterance = self.context["error_handling"]["risposta_corretta"]["utterance"]
                        self.blackboard_scene.face_exp = self.context["error_handling"]["risposta_corretta"]["face"]
                        self.blackboard_scene.gesture = self.context["error_handling"]["risposta_corretta"]["gesture"]
                        self.blackboard_scene.image = self.context["error_handling"]["risposta_corretta"]["image"]
                        self.blackboard_scene.sound = self.context["error_handling"]["risposta_corretta"]["sound"]
                        self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"]["risposta_corretta"]["do_trigger"]=="True"
                    elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.CONFIRM_INTENT.value:
                        print("dialogState == CONFIRM_INTENT")
                        #questo teoricamente serve solo per interaction
                        raise
                    elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.FAILED.value:
                        print("dialogState == FAILED")
                        #siamo qui se: o sono finiti i tentativi oppure se nel confermation intent rispondi no
                        #qui devi dire la risposta corretta
                        #FIXME vedi come prendere il tipo di rifiuto
                        oggetto = self.blackboard_scene.utterance.split(" ")[1]
                        oggetto = "carta"
                        risposta_sbagliata = "risposta_sbagliata_"+oggetto
                        self.blackboard_scene.utterance = self.context["error_handling"][risposta_sbagliata]["utterance"]
                        self.blackboard_scene.face_exp = self.context["error_handling"][risposta_sbagliata]["face"]
                        self.blackboard_scene.gesture = self.context["error_handling"][risposta_sbagliata]["gesture"]
                        self.blackboard_scene.image = self.context["error_handling"][risposta_sbagliata]["image"]
                        self.blackboard_scene.sound = self.context["error_handling"][risposta_sbagliata]["sound"]
                        self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"][risposta_sbagliata]["do_trigger"]=="True"
                        self.blackboard_scene.mainactivity.scene_counter += 1
                    elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.ELICIT_SLOT.value:
                        print("dialogState == ELICIT_SLOT")
                        self.blackboard_scene.utterance = self.context["error_handling"]["sei_sicuro"]["utterance"]
                        self.blackboard_scene.face_exp = self.context["error_handling"]["sei_sicuro"]["face"]
                        self.blackboard_scene.gesture = self.context["error_handling"]["sei_sicuro"]["gesture"]
                        self.blackboard_scene.image = self.context["error_handling"]["sei_sicuro"]["image"]
                        self.blackboard_scene.sound = self.context["error_handling"]["sei_sicuro"]["sound"]
                        self.blackboard_scene.mainactivity.do_trigger = self.context["error_handling"]["sei_sicuro"]["do_trigger"]=="True"
                        self.blackboard_scene.mainactivity.do_kid = True
                    elif self.blackboard_bot.analyzer.result["dialogState"] == DialogStateLex.ELICIT_INTENT.value:
                        print("dialogState == ELICIT_INTENT")
                        self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
                        self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
                        self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
                        self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
                        self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
                        self.blackboard_scene.mainactivity.do_trigger = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["do_trigger"]=="True"
                        self.blackboard_scene.mainactivity.do_kid = True
                    else:
                        print("ELSE")
                        #non dovremmo mai essere in questa parte di codice
                        raise
            self.blackboard_bot.analyzer.result = "null"
        else:
            self.blackboard_scene.utterance = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["utterance"]
            self.blackboard_scene.face_exp = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["face"]
            self.blackboard_scene.gesture = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["gesture"]
            self.blackboard_scene.image = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["image"]
            self.blackboard_scene.sound = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["sound"]
            self.blackboard_scene.mainactivity.do_trigger = self.context["scene"][self.blackboard_scene.mainactivity.scene_counter]["do_trigger"]=="True"
            self.blackboard_scene.mainactivity.scene_counter += 1
            self.counter_non_ho_capito = 0
        self.blackboard_bot.trigger.result = {"message": self.blackboard_scene.utterance}
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