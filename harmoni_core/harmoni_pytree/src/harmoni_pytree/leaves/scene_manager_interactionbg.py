#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

        self.blackboards = []
        self.blackboard_scene = self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.scene.name)
        self.blackboard_scene.register_key(PyTreeNameSpace.interaction.name+"/state", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("utterance", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("face_exp", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key("therapist_needed", access=py_trees.common.Access.WRITE)
        self.blackboard_scene.register_key(PyTreeNameSpace.interaction.name+"/scene_counter", access=py_trees.common.Access.WRITE)
        self.blackboard_bot = self.attach_blackboard_client(name=self.name, namespace=DialogueNameSpace.bot.name)
        self.blackboard_bot.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_stt = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.stt.name)
        self.blackboard_stt.register_key("result", access=py_trees.common.Access.READ)
        self.blackboard_visual= self.attach_blackboard_client(name=self.name, namespace=PyTreeNameSpace.visual.name)
        self.blackboard_visual.register_key("inside", access=py_trees.common.Access.READ)
        self.blackboard_card_detect = self.attach_blackboard_client(name=self.name, namespace=DetectorNameSpace.card_detect.name)
        self.blackboard_card_detect.register_key("result", access=py_trees.common.Access.READ)

        super(SceneManagerInteractionBg, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))


    def setup(self):
        """
        When is this called?
          This function should be either manually called by your program
          to setup this behaviour alone, or more commonly, via
          :meth:`~py_trees.behaviour.Behaviour.setup_with_descendants`
          or :meth:`~py_trees.trees.BehaviourTree.setup`, both of which
          will iterate over this behaviour, it's children (it's children's
          children ...) calling :meth:`~py_trees.behaviour.Behaviour.setup`
          on each in turn.

          If you have vital initialisation necessary to the success
          execution of your behaviour, put a guard in your
          :meth:`~py_trees.behaviour.Behaviour.initialise` method
          to protect against entry without having been setup.

        What to do here?
          Delayed one-time initialisation that would otherwise interfere
          with offline rendering of this behaviour in a tree to dot graph
          or validation of the behaviour's configuration.

          Good examples include:

          - Hardware or driver initialisation
          - Middleware initialisation (e.g. ROS pubs/subs/services)
          - A parallel checking for a valid policy configuration after
            children have been added or removed
        """
        self.logger.debug("  %s [SceneManagerInteractionBg::setup()]" % self.name)

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        self.logger.debug("  %s [SceneManagerInteractionBg::initialise()]" % self.name)

    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        self.logger.debug("  %s [SceneManagerInteractionBg::update()]" % self.name)
        ready_to_make_a_decision = random.choice([True, False])
        decision = random.choice([True, False])
        if not ready_to_make_a_decision:
            return py_trees.common.Status.RUNNING
        elif decision:
            self.feedback_message = "We are not bar!"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Uh oh"
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [SceneManagerInteractionBg::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
