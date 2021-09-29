#!/usr/bin/env python3
##############################################################################
# Imports
##############################################################################

import argparse
import functools
from os import name
from py_trees.behaviours import dummy
import py_trees
import time
from random import randint
import subprocess
import operator
import py_trees.console as console
import running_or_success as rs

import rospy
from harmoni_common_lib.constants import *

from harmoni_pytree.leaves.scene_manager_visualbg import SceneManagerVisualBg
from harmoni_pytree.leaves.subtree_result_visualbg import SubTreeResultVisualBg
from harmoni_pytree.leaves.aws_lex_trigger_service import AWSLexTriggerServicePytree
from harmoni_pytree.leaves.aws_lex_analyzer_service import AWSLexAnalyzerServicePytree
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.facial_exp_service import FacialExpServicePytree
from harmoni_pytree.leaves.lip_sync_service import LipSyncServicePytree
from harmoni_pytree.leaves.google_service import SpeechToTextServicePytree
from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree

##############################################################################
# Classes
##############################################################################

"""
Visual Bg
"""
def description(root):
    content = "\n\n"
    content += "\n"
    content += "EVENTS\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Visual Bg".center(79) + "\n" + console.reset
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

def create_root(name = "Visual_Bg"):
    root = py_trees.composites.Sequence(name = name)

    Success1 = py_trees.behaviours.Success(name="Success")
    Success2 = py_trees.behaviours.Success(name="Success")
    Success3 = py_trees.behaviours.Success(name="Success")

    scene_manager = SceneManagerVisualBg("SceneManagerVisual")
    
    bot_trigger=AWSLexTriggerServicePytree("AwsLexTriggerVisualActivity")
    
    bot_analyzer=AWSLexAnalyzerServicePytree("AwsLexAnalyzerVisualActivity")

    tts = AWSTtsServicePytree("AwsTtsVisualBg")

    stt=SpeechToTextServicePytree("SpeechToTextPytreeVisualBg")

    face_exp=FacialExpServicePytree("FacialExpVisualBg")
 
    speaker=SpeakerServicePytree("SpeakerVisualBg") 

    lip_sync=LipSyncServicePytree("LipSyncVisualBg")

    subtree_result = SubTreeResultVisualBg("SubTreeVisual")

    parall_speaker = py_trees.composites.Parallel(name="ParallelSpeaker")
    parall_speaker.add_children([speaker,lip_sync])    

    sequen_detect_kid = py_trees.composites.Sequence(name="SequenceDetectKid")
    sequen_detect_kid.add_children([stt, bot_analyzer])                                         

    eor_face = py_trees.idioms.either_or(
        name="EitherOrFace",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/face_exp", "null", operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/face_exp", "null", operator.eq),
        ],
        subtrees=[face_exp, Success3],
        namespace="eor_face",
    )

    parall_detect_and_face = py_trees.composites.Parallel(name="ParallelDetectAndFace")
    parall_detect_and_face.add_children([sequen_detect_kid, eor_face])  
    
    eor_trigger = py_trees.idioms.either_or(
        name="EitherOrTrigger",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/" + PyTreeNameSpace.visual.name + "/do_trigger", True, operator.eq),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/" + PyTreeNameSpace.visual.name + "/do_trigger", True, operator.ne),
        ],
        subtrees=[bot_trigger, Success2],
        namespace="eor_trigger",
    )
    sequen_visual = py_trees.composites.Sequence(name="SequenceVisual")
    sequen_visual.add_children([scene_manager, eor_trigger, tts, parall_speaker, parall_detect_and_face])

    eor_visual = py_trees.idioms.either_or(
        name="EitherOrVisual",
        conditions=[
            py_trees.common.ComparisonExpression(DetectorNameSpace.face_detect.name + "/result", "null", operator.eq),
            py_trees.common.ComparisonExpression(DetectorNameSpace.face_detect.name + "/result", "null", operator.ne),
        ],
        subtrees=[sequen_visual, Success1],
        namespace="eor_visual",
    )

    running_or_success = rs.create_root(name="RsVisual", condition=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.visual.name+"/finished", True, operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.visual.name+"/finished", True, operator.eq),
    ])

    root.add_children([eor_visual, subtree_result, running_or_success])

    return root

##############################################################################
# Main
##############################################################################

def render_with_args(root):
    
    args = command_line_argument_parser().parse_args()
    ####################
    # Rendering
    ####################
    if args.render:
        print("**************START RENDERING**************")
        py_trees.display.render_dot_tree(root)
        if py_trees.utilities.which("xdot"):
            try:
                subprocess.call(["xdot", "%s.dot" % root.name])
            except KeyboardInterrupt:
                pass
        else:
            print("")
            console.logerror("No xdot viewer found, skipping display [hint: sudo apt install xdot]")
            print("")
        print("**************END RENDERING**************")
        

def main():
    """
    Entry point for the demo script.
    """
    
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()
    print(description(root))

    #uncomment the following line if you want to render the dot_tree
    #render_with_args(root)
    
    ####################
    # Tree Stewardship
    ####################
    rospy.init_node("visualbg_default", log_level=rospy.INFO)

    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    behaviour_tree.visitors.append(snapshot_visitor)
    additional_parameters = dict([])
    behaviour_tree.setup(timeout=15,**additional_parameters)

    ####################
    # Tick Tock
    ####################

    #if args.interactive:
    #    py_trees.console.read_single_keypress()
    for unused_i in range(1, 50):
        try:
            behaviour_tree.tick()
            #if args.interactive:
            #    py_trees.console.read_single_keypress()
            #else:
            time.sleep(1)
        except KeyboardInterrupt:
            break
    print("\n")

if __name__ == "__main__":
    main()