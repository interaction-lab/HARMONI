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
        self.blackboard_scene.register_key(PyTreeNameSpace.mainactivity.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.mainactivity.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.mainactivity.name+"/max_num_scene", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key(PyTreeNameSpace.mainactivity.name+"/do_kid", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("gesture", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("image", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("sound", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.WRITE)
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

        """
        base_dir = os.getcwd()
        print(base_dir)
        
        with open(
            base_dir + "/../../../resources/mainactivity.json", "r"
        ) as json_file:
            self.context = json.load(json_file)
        """

        pattern_name = "mainactivity"
        rospack = rospkg.RosPack()
        pck_path = rospack.get_path("harmoni_pytree")
        pattern_script_path = pck_path + f"/resources/{pattern_name}.json"
        with open(pattern_script_path, "r") as read_file:
          self.context = json.load(read_file)
        
        #per interagire con context fai una cosa simile a questa riga sotto
        #bb = contex["scene"][counter_scene]["gesture"]

        #print("Lunghezza: " + len(self.context["scene"]))
        self.blackboard_scene.mainactivity.max_num_scene = len(self.context["scene"])
        
        #print("TEST: context gesture is  %s " % self.context["scene"][self.scene_counter]["gesture"])

        self.logger.debug("  %s [SceneManagerMain::setup()]" % self.name)

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        self.logger.debug("  %s [SceneManagerMain::initialise()]" % self.name)

    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug("  %s [SceneManagerMain::update()]" % self.name)
        
        if self.blackboard_visual.inside == True:
          self.blackboard_visual.inside = False
          #TODO gestisci visual ture, forse vogliamo fare un intent per far dire al robot "Okay riprendiamo l'attività" 
                                      #e poi ripeti l'ultima scena che avevamo fatto
        if self.blackboard_interaction.inside == True:
          self.blackboard_interaction.inside = False
          #TODO gestisci interaction ture ovvero ripeti l'ultima scena
        """
        else if intent_raggiunto:
          self.scene_counter += 1
          setta tutte le bb con quello che sta dentro context
        else if:
          se è partito intent stop --> 
            self.blackboard_scene.therapist_needed = True
            fai partire intent terapista
        else if:
          se è partito intent nocapito --> 
            se non ha capito 2 volte di seguito -->
              self.blackboard_scene.therapist_needed = True
              fai partire intent terapista
            eltrimenti -->
              setta tutte le bb con quello che sta dentro context
        else if:
          se hai sbagliato due volte di seguito salta alla prossima scena altrimenti fai -->
          #TODO intent sbagliato, far dire una cosa leggermente diversa, sei sicuro? dovre butteresti questo?
        else if:
          scene_counter == 0 -->
          setta tutte le bb con quello che sta dentro context
        """
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
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