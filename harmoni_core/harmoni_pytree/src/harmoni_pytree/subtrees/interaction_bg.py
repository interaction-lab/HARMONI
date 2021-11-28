#!/usr/bin/env python3
##############################################################################
# Imports
##############################################################################

import argparse
import functools
from py_trees.behaviours import dummy
from py_trees.idioms import either_or
import py_trees
import rospy
import time
import subprocess
import operator
import py_trees.console as console

from harmoni_common_lib.constants import *

from harmoni_pytree.leaves.buttons import Buttons
from harmoni_pytree.leaves.aws_lex_trigger_service import AWSLexTriggerServicePytree
from harmoni_pytree.leaves.aws_lex_analyzer_service import AWSLexAnalyzerServicePytree
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.facial_exp_service import FacialExpServicePytree
from harmoni_pytree.leaves.lip_sync_service import LipSyncServicePytree
from harmoni_pytree.leaves.google_service import SpeechToTextServicePytree
from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree
from harmoni_pytree.leaves.custom_yolo_service import ImageAICustomServicePytree
from harmoni_pytree.leaves.scene_manager_interactionbg import SceneManagerInteractionBg
from harmoni_pytree.leaves.subtree_result_interaction import SubTreeResultInteractionBg

##############################################################################
# Classes
##############################################################################
"""
Interaction Bg
"""

def description(root):
    content = "\n\n"
    content += "\n"
    content += "EVENTS\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Interaction_Bg".center(79) + "\n" + console.reset
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

def create_root(name = "Interaction_Bg"):
    root = py_trees.composites.Sequence(name=name)

    Success1 = py_trees.behaviours.Success(name="Success")
    Success2 = py_trees.behaviours.Success(name="Success")
    Success3 = py_trees.behaviours.Success(name="Success")
    Success4 = py_trees.behaviours.Success(name="Success")
    Success5 = py_trees.behaviours.Success(name="Success")
    Success6 = py_trees.behaviours.Success(name="Success")
    Success7 = py_trees.behaviours.Success(name="Success")


    scene_manager = SceneManagerInteractionBg("SceneManagerInteractionBg")

    bot_trigger=AWSLexTriggerServicePytree("AwsLexTriggerInteractionActivity")

    bot_analyzer=AWSLexAnalyzerServicePytree("AwsLexAnalyzerInteractionActivity")

    tts = AWSTtsServicePytree("AwsTtsInteractionBg")

    stt = SpeechToTextServicePytree("SpeechToTextInteractionBg")

    buttons = Buttons(name="ButtonsInteraction")
       
    custom_yolo = ImageAICustomServicePytree("DetectionCardInteraction")

    face_exp=FacialExpServicePytree("FacialExpInteractionBg")

    speaker=SpeakerServicePytree("SpeakerInteractionBg")

    lip_sync=LipSyncServicePytree("LipSyncInteractionBg")
                                             
    subtree_result=SubTreeResultInteractionBg("SubTreeInteraction")

    parall_speaker = py_trees.composites.Parallel(name="ParallelSpeaker")
    parall_speaker.add_children([speaker,lip_sync])  

    parall_detect_kid = py_trees.composites.Parallel(name="ParallelDetectKid")
    parall_detect_kid.add_children([stt, custom_yolo, buttons])

    inverter_parall_detect_kid = py_trees.decorators.Inverter(name="ParallelDetectKidInverter",child=parall_detect_kid)

    periodic_success = py_trees.decorators.FailureIsRunning(child=py_trees.behaviours.SuccessEveryN(name="SuccessEveryN",n=3),name="FIsRSEN") 

    eor_null_button = py_trees.idioms.either_or(
        name="EitherOrNullButton",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.buttons.name+ "/result", "null", operator.eq),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.buttons.name+ "/result", "null", operator.ne),
        ],
        subtrees=[periodic_success,Success7],
        namespace="eor_null_button",
    )
    eor_null_speech = py_trees.idioms.either_or(
        name="EitherOrNullSpeech",
        conditions=[
            py_trees.common.ComparisonExpression(DetectorNameSpace.stt.name+ "/result", "null", operator.eq),
            py_trees.common.ComparisonExpression(DetectorNameSpace.stt.name+ "/result", "null", operator.ne),
        ],
        subtrees=[eor_null_button,Success6],
        namespace="eor_null_speech",
    )
    eor_null_card = py_trees.idioms.either_or(
        name="EitherOrNullCard",
        conditions=[
            py_trees.common.ComparisonExpression(DetectorNameSpace.card_detect.name+ "/result", "null", operator.eq),
            py_trees.common.ComparisonExpression(DetectorNameSpace.card_detect.name+ "/result", "null", operator.ne),
        ],
        subtrees=[eor_null_speech,Success5],
        namespace="eor_null_card",
    )

    selector_detect_kid = py_trees.composites.Selector(name="SelectorDetectKid")
    selector_detect_kid.add_children([inverter_parall_detect_kid, eor_null_card])

    sequen_detect_kid = py_trees.composites.Sequence(name="SequenceDetectKid")
    sequen_detect_kid.add_children([selector_detect_kid, bot_analyzer])                                         

    eor_face = py_trees.idioms.either_or(
        name="EitherOrFace",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/face_exp", "null", operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/face_exp", "null", operator.eq),
        ],
        subtrees=[face_exp, Success3],
        namespace="eor_face",
    )

    eor_kid = py_trees.idioms.either_or(
        name="EitherOrKid",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/"+ PyTreeNameSpace.interaction.name+ "/do_kid", True, operator.eq),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/"+ PyTreeNameSpace.interaction.name+ "/do_kid", True, operator.ne),
        ],
        subtrees=[sequen_detect_kid, Success4],
        namespace="eor_kid",
    )

    parall_detect_and_face = py_trees.composites.Parallel(name="ParallelDetectAndFace")
    parall_detect_and_face.add_children([eor_kid, eor_face])  

    sequen_interaction_bg = py_trees.composites.Sequence(name="SequenceInteractionBg")

    eor_bot_trigger = py_trees.idioms.either_or(
        name="EitherOrBotTrigger",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/" + PyTreeNameSpace.interaction.name + "/do_trigger", True, operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/" + PyTreeNameSpace.interaction.name + "/do_trigger", True, operator.eq),
        ],
        subtrees=[Success2, bot_trigger],
        namespace="eor_bot_trigger",
    )

    sequen_interaction_bg.add_children([scene_manager,
                                        eor_bot_trigger, 
                                        tts, 
                                        parall_speaker, 
                                        parall_detect_and_face])  

    eor_interaction_bg = either_or(
        name="Either_Or_Interaction_Bg",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.mainactivity.name+"/counter_no_answer", 2, operator.lt),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.mainactivity.name+"/counter_no_answer", 2, operator.ge),
        ],
        subtrees=[Success1,sequen_interaction_bg],
        namespace="eor_interaction_bg",
    )

    root.add_children([eor_interaction_bg, subtree_result])

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
    #print(description(root))
    #uncomment the following line if you want to render the dot_tree
    #render_with_args()

    rospy.init_node("interactionbg_default", log_level=rospy.INFO)

    ####################
    # Tree Stewardship
    ####################
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

    #if args.interactive:
    #    py_trees.console.read_single_keypress()
    for unused_i in range(1, 30):
        try:
            behaviour_tree.tick()
            #if args.interactive:
            #    py_trees.console.read_single_keypress()
            #else:
            time.sleep(0.4)
        except KeyboardInterrupt:
            break
    print("\n")


if __name__ == "__main__":
    main()