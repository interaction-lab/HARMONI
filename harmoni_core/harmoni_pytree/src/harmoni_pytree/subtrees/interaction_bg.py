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
import subprocess
import operator
import py_trees.console as console
import either_custom as eu
import running_or_success as rs

from harmoni_pytree.leaves.aws_lex_service_pytree import AWSLexServicePytree
from harmoni_pytree.leaves.aws_tts_service_pytree import AWSTtsServicePytree
from harmoni_pytree.leaves.face_service_pytree import FaceServicePytree
from harmoni_pytree.leaves.google_service_pytree import SpeechToTextServicePytree
from harmoni_pytree.leaves.microphone_service_pytree import MicrophoneServicePytree
from harmoni_pytree.leaves.speaker_service_pytree import SpeakerServicePytree
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
    
    Success = py_trees.behaviours.Success(name="Success")

    #TODO modulo sceneManager!
    scene_manager = SceneManagerInteractionBg("SceneManagerInteractionBg")
    """
    Interaction_Bg_Scene = py_trees.behaviours.SetBlackboardVariable(name="Interaction_Bg_Scene(do_speech)",
                                                        variable_name="do_speech", 
                                                        variable_value="null", 
                                                        overwrite=True)
    #dummy1 è da sostituire con la variabile face_exp nel scene.
    dummy1 = py_trees.behaviours.SetBlackboardVariable(name="do_face",
                                                        variable_name="do_face", 
                                                        variable_value="null", 
                                                        overwrite=True)
    """
    chatbot = AWSLexServicePytree("AwsLexInteractionBg")
    chatbot2 = AWSLexServicePytree("AwsLexInteractionBg2")
    """                                                    
    Chat_Bot = py_trees.behaviours.Count(name="Chat_Bot",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    tts = AWSTtsServicePytree("AwsTtsInteractionBg")
    """                                                
    Tts = py_trees.behaviours.Count(name="Tts",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    stt = SpeechToTextServicePytree("SpeechToTextInteractionBg")
    """                                            
    Stt = py_trees.behaviours.Count(name="Stt",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    #TODO mancano le foglie di imageAi         
    custom_yolo = CustomYoloServicePytree("DetectionCardInteraction")
    """                                      
    Detection_Card = py_trees.behaviours.Count(name="Detection_Card",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    face_exp=FaceServicePytree("FaceInteractionBg")
    """                                                  
    Facial_Expression = py_trees.behaviours.Count(name="Facial_Expression",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    speaker=SpeakerServicePytree("SpeakerInteractionBg")
    """                                                  
    Speaker = py_trees.behaviours.Count(name="Speaker",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    lips_sync=FaceServicePytree("LipsSyncInteractionBg")
    """                                                  
    Lips_Synk = py_trees.behaviours.Count(name="Lips_Synk",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    microphone=MicrophoneServicePytree("MicrophoneInteractionBg")
    """                                                
    Microphone = py_trees.behaviours.Count(name="Microphone",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """                                               
    invalid_response_stt = py_trees.behaviours.SetBlackboardVariable(name="InvalidResponseIntStt",
                                                        variable_name=DetectorNameSpace.stt.name+"/result", 
                                                        variable_value="null", 
                                                        overwrite=True)
    invalid_response_card = py_trees.behaviours.SetBlackboardVariable(name="InvalidResponseIntCard",
                                                        variable_name=DetectorNameSpace.card_detect.name+"/result", 
                                                        variable_value="null", 
                                                        overwrite=True)
    #TODO timer                                                  
    timeout_kid_detection = Timer(name="TimeoutKidDetectionInt",
                                                    variable_namespace=PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.interaction.name, 
                                                    variable_name="kid_detection")
    #TODO modulo per vedere se il sottoalbero è terminato
    subtree_result=SubTreeResultInteractionBg("SubTreeInteraction")
    """                                              
    Interaction_Bg_Subtree_Results = py_trees.behaviours.Count(name="Interaction_Bg_Subtree_Results",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """

    parall_speaker = py_trees.composites.Parallel(name="ParallelSpeaker")
    parall_speaker.add_children([speaker,lips_sync])  

    sequen_speech_kid = py_trees.composites.Sequence(name="SequenceSpeechKid")
    sequen_speech_kid.add_children([microphone ,stt])

    parall_detect_kid = py_trees.composites.Parallel(name="ParallelDetectKid")
    parall_detect_kid.add_children([sequen_speech_kid,custom_yolo])

    sequen_invalid_response = py_trees.composites.Sequence(name="SequenceInvalid")
    sequen_invalid_response.add_children([invalid_response_stt, invalid_response_card])

    eor_timer_detection = either_custom.either_or(
        name="EitherOrTimerDetection",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.interaction.name + "/kid_detection", 10, operator.lt),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.interaction.name + "/kid_detection", 10, operator.ge),
        ],
        preemptible = False,
        subtrees=[parall_detect_kid, sequen_invalid_response],
        namespace="eor_timer_detection",
    )

    sequen_detect_kid = py_trees.composites.Sequence(name="SequenceDetectKid",memory=False)
    sequen_detect_kid.add_children([timeout_kid_detection, eor_timer_detection, chatbot2])                                         

    parall_detect_and_face = py_trees.composites.Parallel(name="ParallelDetectAndFace")
    parall_detect_and_face.add_children([sequen_detect_kid, face_exp])  

    sequen_interaction_bg = py_trees.composites.Sequence(name="SequenceInteractionBg")

    sequen_interaction_bg.add_children([scene_manager,
                                        chatbot, 
                                        tts, 
                                        parall_speaker, 
                                        parall_detect_and_face])  

    eor_interaction_bg = either_custom.either_or(
        name="Either_Or_Interaction_Bg",
        conditions=[
            py_trees.common.ComparisonExpression(PyTreeNameSpace.invalid_response.name+"/"+PyTreeNameSpace.mainactivity.name+"/counter_no_answer", 2, operator.lt),
            py_trees.common.ComparisonExpression(PyTreeNameSpace.invalid_response.name+"/"+PyTreeNameSpace.mainactivity.name+"/counter_no_answer", 2, operator.ge),
        ],
        preemptible = False,
        subtrees=[Success, sequen_interaction_bg],
        namespace="eor_interaction_bg",
    )

    running_or_success = rs.create_root()

    root.add_children([eor_interaction_bg, subtree_result, running_or_success])

    return root

##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    args = command_line_argument_parser().parse_args()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()
    print(description(root))

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

    if args.interactive:
        py_trees.console.read_single_keypress()
    for unused_i in range(1, 10):
        try:
            behaviour_tree.tick()
            if args.interactive:
                py_trees.console.read_single_keypress()
            else:
                time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")


if __name__ == "__main__":
    main()