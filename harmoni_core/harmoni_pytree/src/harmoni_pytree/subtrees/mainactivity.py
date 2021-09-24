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
"""
Main Activity
"""
def description(root):
    content = "\n\n"
    content += "\n"
    content += "EVENTS\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Main Activity".center(79) + "\n" + console.reset
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

def create_root():
    root = py_trees.composites.Sequence(name="mainactivity",memory=True)
    
    blackboard_scene_mainactivity = root.attach_blackboard_client(name="mainactivity", namespace=PyTreeNameSpace.scene.name +"/"+ PyTreeNameSpace.mainactivity.name)
    blackboard_scene_mainactivity.register_key("max_num_scene", access=py_trees.common.Access.WRITE)
    blackboard_scene_mainactivity.max_num_scene = 0
    
    blackboard_visual = root.attach_blackboard_client(name="self.name", namespace=PyTreeNameSpace.visual.name)
    blackboard_visual.register_key("inside", access=py_trees.common.Access.WRITE)
    blackboard_visual.inside = False
    blackboard_interaction = root.attach_blackboard_client(name="self.name", namespace=PyTreeNameSpace.interaction.name)
    blackboard_interaction.register_key("inside", access=py_trees.common.Access.WRITE)
    blackboard_interaction.inside = False

    Success1 = py_trees.behaviours.Success(name="Success")
    Success2 = py_trees.behaviours.Success(name="Success")
    Success3 = py_trees.behaviours.Success(name="Success")
    Success4 = py_trees.behaviours.Success(name="Success")
    Success5 = py_trees.behaviours.Success(name="Success")
    Success6 = py_trees.behaviours.Success(name="Success")
    Success7 = py_trees.behaviours.Success(name="Success")

    scene_manager = SceneManagerMain("SceneManagerMain")

    #gesture=GestureServicePytree("GestureMainActivity")                                                 
    gesture = py_trees.behaviours.Success(name="GestureMainActivity")
    
    #TODO sostituirlo/capire se si può usare web_service.
    Projector = py_trees.behaviours.Success(name="Projector")

    ext_speaker=SpeakerServicePytree("ExternalSpeakerMainActivity")

    bot_trigger=AWSLexTriggerServicePytree("AwsLexTriggerMainActivity")

    bot_analyzer=AWSLexAnalyzerServicePytree("AwsLexAnalyzerMainActivity")

    tts = AWSTtsServicePytree("AwsTtsMainActivity")

    stt=SpeechToTextServicePytree("SpeechToTextMainActivity")

    face_exp=FacialExpServicePytree("FacialExpMainActivity")

    speaker=SpeakerServicePytree("SpeakerMainActivity") 

    lip_sync=LipSyncServicePytree("LipsSyncMainActivity")

    microphone=MicrophoneServicePytree("MicrophoneMainActivity")

    custom_yolo = ImageAICustomServicePytree("DetectionCardMain")
                                               
    invalid_response_stt = py_trees.behaviours.SetBlackboardVariable(name="InvalidResponseMainStt",
                                                        variable_name=DetectorNameSpace.stt.name+"/result", 
                                                        variable_value="null", 
                                                        overwrite=True)
    invalid_response_card = py_trees.behaviours.SetBlackboardVariable(name="InvalidResponseMainCard",
                                                        variable_name=DetectorNameSpace.card_detect.name+"/result", 
                                                        variable_value="null", 
                                                        overwrite=True)
                                        
    counter_no_answer = CounterNoAnswer(name="CounterNoAnswer",
                                        variable_name= PyTreeNameSpace.invalid_response.name+"/"+PyTreeNameSpace.mainactivity.name+"/counter_no_answer") 
                                             
    timer_kid_detection = Timer(name="TimerKidDetectionInt",
                                variable_name=PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.mainactivity.name+"/kid_detection",
                                duration = 10)
    timer_reset = TimerReset(name="TimerResetKidDetectionInt",
                            variable_name=PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.mainactivity.name+"/kid_detection")                                               
    
    subtree_result = SubTreeResultMain("SubTreeResultMain")

    parall_speaker = py_trees.composites.Parallel(name="ParallelSpeaker")
    parall_speaker.add_children([speaker,lip_sync]) 

    eor_trigger = py_trees.idioms.either_or(
        name="EitherOrTrigger",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/" + PyTreeNameSpace.mainactivity.name + "/do_dialogue", "yes", operator.eq),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/" + PyTreeNameSpace.mainactivity.name + "/do_dialogue", "yes", operator.ne),
        ],
        subtrees=[bot_trigger, Success7],
        namespace="eor_trigger",
    )

    sequen_speaker = py_trees.composites.Sequence(name="SequenceSpeaker")
    sequen_speaker.add_children([eor_trigger,tts,parall_speaker])

    eor_speaker = py_trees.idioms.either_or(
        name="EitherOrSpeaker",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/utterance", "null", operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/utterance", "null", operator.eq),
        ],
        subtrees=[sequen_speaker, Success1],
        namespace="eor_speaker",
    )
    eor_face = py_trees.idioms.either_or(
        name="EitherOrFace",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/face", "null", operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/face", "null", operator.eq),
        ],
        subtrees=[face_exp, Success5],
        namespace="eor_face",
    )
    eor_gesture = py_trees.idioms.either_or(
        name="EitherOrGesture",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/gesture", "null", operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/gesture", "null", operator.eq),
        ],
        subtrees=[gesture, Success3],
        namespace="eor_gesture",
    )
    eor_external_speaker = py_trees.idioms.either_or(
        name="EitherOrExternalSpeaker",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/sound", "null", operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/sound", "null", operator.eq),
        ],
        subtrees=[ext_speaker, Success4],
        namespace="eor_external_speaker",
    )

    parall_face_and_gesture = py_trees.composites.Parallel(name="ParallelFaceAndGesture")
    parall_face_and_gesture.add_children([eor_face,eor_gesture])

    sequen_speaker_and_parallel_f_g = py_trees.composites.Sequence(name="SequenceSpeakerAndParallelFG")
    sequen_speaker_and_parallel_f_g.add_children([eor_speaker,parall_face_and_gesture])
    
    parall_robot = py_trees.composites.Parallel(name="ParallelRobot")
    parall_robot.add_children([eor_external_speaker, sequen_speaker_and_parallel_f_g])
    
    sequen_robot = py_trees.composites.Sequence(name="SequenceRobot")
    sequen_robot.add_children([scene_manager,Projector,parall_robot])

    sequen_speech_kid = py_trees.composites.Sequence(name="SequenceSpeechKid")
    sequen_speech_kid.add_children([microphone ,stt])

    parall_detect_kid = py_trees.composites.Parallel(name="ParallelDetectKid")
    parall_detect_kid.add_children([sequen_speech_kid,custom_yolo])

    sequence_invalid_response = py_trees.composites.Sequence(name="SequenceInvalidResponse")
    sequence_invalid_response.add_children([invalid_response_stt, invalid_response_card])

    eor_timer_detection = eu.either_or(
        name="EitherOrTimerDetection",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.visual.name + "/kid_detection", 10, operator.lt),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.visual.name + "/kid_detection", 10, operator.ge),
        ],
        preemptible = False,
        subtrees=[parall_detect_kid, sequence_invalid_response],
        namespace="eor_timer_detection",
    )

    sequen_detect_kid = py_trees.composites.Sequence(name="SequenceDetectKid")
    sequen_detect_kid.add_children([timer_kid_detection, eor_timer_detection, timer_reset, bot_analyzer])                                         

    running_or_success = rs.create_root(name="RsMainactivity", condition=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.mainactivity.name+"/finished", "True", operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.mainactivity.name+"/finished", "True", operator.eq),
    ])

    eor_kid = py_trees.idioms.either_or(
        name="EitherOrKid",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/do_dialogue", "null", operator.ne),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.scene.name + "/do_dialogue", "null", operator.eq),
        ],
        subtrees=[sequen_detect_kid, Success2],
        namespace="eor_kid",
    )

    root.add_children([sequen_robot, eor_kid, subtree_result, running_or_success])

    return root

##############################################################################
# Main
##############################################################################

def render_with_args():
    
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
    #render_with_args()
        
    ####################
    # Tree Stewardship
    ####################

    rospy.init_node("mainactivity_default", log_level=rospy.INFO)

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
    for unused_i in range(1, 10):
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