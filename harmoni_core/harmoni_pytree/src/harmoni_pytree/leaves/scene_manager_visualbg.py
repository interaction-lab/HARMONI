#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
import random


class SceneManagerVisualBg(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """

        self.scene_counter = 0

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.visual.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.READ)
        """
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        """
        self.blackboard_invalid_mainactivuty = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.invalid_response.name +"/"+ PyTreeNameSpace.mainactivity.name)
        self.blackboard_invalid_mainactivuty.register_key("counter_no_answer", access=py_trees.common.Access.READ)

        super(SceneManagerVisualBg, self).__init__(name)
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
        self.logger.debug("  %s [SceneManagerVisualBg::setup()]" % self.name)

    def initialise(self):

        self.logger.debug("  %s [SceneManagerVisualBg::initialise()]" % self.name)

    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug("  %s [SceneManagerVisualBg::update()]" % self.name)
        
        self.blackboard_invalid_mainactivuty.counter_no_answer = 0

        if intent raggiunto:
          self.scene_counter += 1
          setta tutte le bb con quello che sta dentro context
        else if:
          è la seconda volta che fai questo BG --> 
            self.blackboard_scene.therapist_needed = True
            fai partire intent terapista
        else if:
          scene_counter == 0 -->
          setta tutte le bb con quello che sta dentro context

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [SceneManagerVisualBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))