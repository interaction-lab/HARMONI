#!/usr/bin/env python3
##############################################################################
# Imports
##############################################################################

import argparse
import functools
from os import name
from py_trees.behaviours import dummy
from py_trees.idioms import either_or
import rospy
import py_trees
import time
from random import randint
import subprocess
import operator
import py_trees.console as console
import running_or_success as rs

from harmoni_common_lib.constants import *

from harmoni_pytree import either_custom
from harmoni_pytree.leaves.timer import Timer
from harmoni_pytree.leaves.scene_manager_visualbg import SceneManagerVisualBg
from harmoni_pytree.leaves.subtree_result_visualbg import SubTreeResultVisualBg
from harmoni_pytree.leaves.aws_lex_service import AWSLexServicePytree
from harmoni_pytree.leaves.aws_tts_service import AWSTtsServicePytree
from harmoni_pytree.leaves.face_service import FaceServicePytree
from harmoni_pytree.leaves.google_service import SpeechToTextServicePytree
from harmoni_pytree.leaves.microphone_service import MicrophoneServicePytree
from harmoni_pytree.leaves.speaker_service import SpeakerServicePytree
from harmoni_pytree.leaves.yolo_service import ImageAIYoloServicePytree

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
    
    Success = py_trees.behaviours.Success(name="Success")

    #TODO modulo sceneManager!
    scene_manager = SceneManagerVisualBg("SceneManagerVisual")
    """
    Visual_Bg_Scene = py_trees.behaviours.SetBlackboardVariable(name="Visual_Bg_Scene(do_speech)",
                                                        variable_name="scene/chatbot_in_namespace/result_data", 
                                                        variable_value="non vedo il bimbo (intent)", 
                                                        overwrite=True)
    #dummy1 è da sostituire con la variabile face_exp nel scene. 
    dummy1 = py_trees.behaviours.SetBlackboardVariable(name="do_face",
                                                        variable_name="scene/face_bb_namespace/result_data", 
                                                        variable_value="Triste", 
                                                        overwrite=True)
    """
    chatbot = AWSLexServicePytree("AwsLexVisualBg")
    chatbot2 = AWSLexServicePytree("AwsLexVisualBg2")
    """                                                    
    Chat_Bot = py_trees.behaviours.SetBlackboardVariable(name="Chat_bot",
                                                        variable_name="bot_output_bb_namespace/result_data", 
                                                        variable_value="Non ti vedo, dove sei?", 
                                                       overwrite=True)
    """
    tts = AWSTtsServicePytree("AwsTtsVisualBg")
    """
    Tts = py_trees.behaviours.Count(name="Tts",
                                        fail_until=0,
                                        running_until=1,
                                        success_until=10,
                                        reset=False)
    """
    stt=SpeechToTextServicePytree("SpeechToTextPytreeVisualBg")
    """
    Stt = py_trees.behaviours.Count(name="Stt",
                                        fail_until=0,
                                        running_until=1,
                                        success_until=10,
                                        reset=False)
    """
    face_exp=FaceServicePytree("FaceVisualBg")
    """
    Facial_Expression = py_trees.behaviours.Count(name="Facial_Expression",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """ 
    speaker=SpeakerServicePytree("SpeakerVisualBg") 
    """                                                
    Speaker = py_trees.behaviours.Count(name="Speaker",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    lips_sync=FaceServicePytree("LipsSyncVisualBg")
    """                                                  
    Lips_Synk = py_trees.behaviours.Count(name="Lips_Synk",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    microphone=MicrophoneServicePytree("MicrophonePytreeVisualBg")
    """                                                  
    Microphone = py_trees.behaviours.Count(name="Microphone",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    #TODO mancano le foglie di imageAi 
    yolo_service = ImageAIYoloServicePytree("FaceDetection")
    """                                           
    Detection_Face = py_trees.behaviours.SetBlackboardVariable(name="Detection_Face",
                                                        variable_name="scene/detection_face_bb_namespace/result_message", 
                                                        variable_value="non null", 
                                                        overwrite=True)
    """
    invalid_response = py_trees.behaviours.SetBlackboardVariable(name="invalid_response_visual_stt",
                                                        variable_name=DetectorNameSpace.stt.name+"/result", 
                                                        variable_value="null", 
                                                        overwrite=True)
    #TODO timer
    timeout_kid_detection = Timer(name="TimeoutKidDetectionVis",
                                                    variable_namespace=PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.visual.name, 
                                                    variable_name="kid_detection")
    #TODO modulo per vedere se il sottoalbero è terminato
    subtree_result = SubTreeResultVisualBg("SubTreeVisual")
    """
    Visual_Bg_Subtree_Results = py_trees.behaviours.Count(name="Visual_Bg_Subtree_Results",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    parall_speaker = py_trees.composites.Parallel(name="ParallelSpeaker")
    parall_speaker.add_children([speaker,lips_sync])    

    sequen_speech_kid = py_trees.composites.Sequence(name="SequenceSpeechKid")
    sequen_speech_kid.add_children([microphone ,stt])

    eor_timer_detection = either_custom.either_or(
        name="EitherOrTimerDetection",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.visual.name + "/kid_detection", 10, operator.lt),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.visual.name + "/kid_detection", 10, operator.ge),
        ],
        preemptible = False,
        subtrees=[sequen_speech_kid, invalid_response],
        namespace="eor_timer_detection",
    )

    sequen_detect_kid = py_trees.composites.Sequence(name="SequenceDetectKid",memory=False)
    sequen_detect_kid.add_children([timeout_kid_detection, eor_timer_detection, chatbot2])                                         

    parall_detect_and_face = py_trees.composites.Parallel(name="ParallelDetectAndFace")
    parall_detect_and_face.add_children([sequen_detect_kid, face_exp])  

    sequen_visual = py_trees.composites.Sequence(name="SequenceVisual")
    sequen_visual.add_children([scene_manager,chatbot,tts,parall_speaker,parall_detect_and_face])

    eor_visual = either_custom.either_or(
        name="EitherOrVisual",
        conditions=[
            py_trees.common.ComparisonExpression(DetectorNameSpace.face_detect.name + "/result", "null", operator.ne),
            py_trees.common.ComparisonExpression(DetectorNameSpace.face_detect.name + "/result", "null", operator.eq),
        ],
        preemptible = False,
        subtrees=[Success, sequen_visual],
        namespace="eor_visual",
    )
    #TODO you have to change condition
    running_or_success = rs.create_root(name="rs_visual",condition=[
            py_trees.common.ComparisonExpression("/change", "change", operator.ne),
            py_trees.common.ComparisonExpression("/change", "change", operator.eq),
        ])

    root.add_children([yolo_service, eor_visual, subtree_result, running_or_success])

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
    for unused_i in range(1, 10):
        try:
            behaviour_tree.tick()
            #if args.interactive:
            #    py_trees.console.read_single_keypress()
            #else:
            time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")

if __name__ == "__main__":
    main()