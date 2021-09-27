#!/usr/bin/env python3
##############################################################################
# Imports
##############################################################################

import argparse
import functools
from py_trees.behaviours import dummy
from py_trees.idioms import either_or
import py_trees
import time
import rospy
from random import randint
import subprocess
import operator
import py_trees.console as console
from harmoni_pytree import either_custom as eu
import running_or_success as rs

from harmoni_common_lib.constants import *

from harmoni_pytree.leaves.aws_lex_trigger_service import AWSLexTriggerServicePytree
from harmoni_pytree.leaves.aws_lex_analyzer_service import AWSLexAnalyzerServicePytree
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.facial_exp_service import FacialExpServicePytree
from harmoni_pytree.leaves.lip_sync_service import LipSyncServicePytree
from harmoni_pytree.leaves.google_service import SpeechToTextServicePytree
from harmoni_pytree.leaves.microphone_service import MicrophoneServicePytree
from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree
#from harmoni_pytree.leaves.gesture_service import GestureServicePytree
from harmoni_pytree.leaves.subtree_result_main import SubTreeResultMain
from harmoni_pytree.leaves.counter_no_answer import CounterNoAnswer
from harmoni_pytree.leaves.scene_manager_main import SceneManagerMain
from harmoni_pytree.leaves.custom_yolo_service import ImageAICustomServicePytree
from harmoni_pytree.leaves.timer import Timer
from harmoni_pytree.leaves.timer_reset import TimerReset

##############################################################################
# Classes
##############################################################################


def description(root):
    content = "\n\n"
    content += "\n"
    content += "EVENTS\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Test".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(create_root()),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-b', '--with-blackboard-variables', default=False, action='store_true', help='add nodes for the blackboard variables')
    group.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    group.add_argument('-i', '--interactive', action='store_true', help='pause and wait for keypress at each tick')
    return parser


def pre_tick_handler(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def post_tick_handler(snapshot_visitor, behaviour_tree):
    print(
        "\n" + py_trees.display.unicode_tree(
            root=behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited
        )
    )
    print(py_trees.display.unicode_blackboard())


def create_root(name= "MicAndSTT"):

    microphone=MicrophoneServicePytree("MicrophoneMainActivity")
    stt=SpeechToTextServicePytree("SpeechToTextMainActivity")

    root = py_trees.composites.Sequence(name="MicAndSTT")
    root.add_children([microphone ,stt])

    return root

##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()
    print(description(root))

    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace=DetectorNameSpace.stt.name)
    blackboardProva.register_key("result", access=py_trees.common.Access.READ)
    print(blackboardProva)
        
    ####################
    # Tree Stewardship
    ####################

    rospy.init_node("test_default", log_level=rospy.INFO)

    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    behaviour_tree.visitors.append(snapshot_visitor)
    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################

    for unused_i in range(1, 30):
        try:
            behaviour_tree.tick()
            time.sleep(3)
        except KeyboardInterrupt:
            break
    print("\n")


if __name__ == "__main__":
    main()