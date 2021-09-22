#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from harmoni_common_lib.constants import *
import py_trees
import random


class SceneManagerInteractionBg(py_trees.behaviour.Behaviour):
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
        self.blackboard_scene.register_key(PyTreeNameSpace.interaction.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.interaction.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.interaction.name+"/max_num_scene", access=py_trees.common.Access.WRITE) #NEW
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_visual= self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key("inside", access=py_trees.common.Access.READ)
        super(SceneManagerInteractionBg, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))


    def setup(self):
        #TODO load all the utterance in a varaible
        # number_of_scene(0):
        #                     utterance:
        #                     gesture:
        #                     .
        #                     .
        #                     .
        #                 1:
        #                     utterance:
        #                     gesture:
        #                     ...
        #                 .
        #                 .
        #                 .
        #context[number_of_scene][utterance]
        self.logger.debug("  %s [SceneManagerInteractionBg::setup()]" % self.name)

        self.blackboard_scene.interaction.max_num_scene = len(self.context["scene"])


    def initialise(self):
        self.logger.debug("  %s [SceneManagerInteractionBg::initialise()]" % self.name)

    def update(self):

        self.logger.debug("  %s [SceneManagerInteractionBg::update()]" % self.name)
        """
        if intent raggiunto:
          self.scene_counter += 1
          setta tutte le bb con quello che sta dentro context
        else if:
          se è partito intent stop --> 
            self.blackboard_scene.therapist_needed = True
            fai partire intent terapista
        else if:
          se risponde che vuole interrompere l'attività
            self.blackboard_scene.therapist_needed = True
            fai partire intent terapista
        else if intent non raggiunto 
          self.blackboard_scene.therapist_needed = True
            fai partire intent terapista
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
        self.logger.debug("  %s [SceneManagerInteractionBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
