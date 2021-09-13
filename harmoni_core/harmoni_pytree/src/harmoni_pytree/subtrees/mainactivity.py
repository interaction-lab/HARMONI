##############################################################################
# Imports
##############################################################################

import argparse
import functools
from py_trees.behaviours import dummy
from py_trees.idioms import either_or
import py_trees
import time
from random import randint
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
from harmoni_pytree.leaves.gesture_service_pytree import GestureServicePytree
from harmoni_pytree.leaves.counter_no_answer import CounterNoAnswer
from harmoni_pytree.leaves.subtree_result_main import SubTreeResultMain
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
    
    Success1 = py_trees.behaviours.Success(name="Success")
    Success2 = py_trees.behaviours.Success(name="Success")
    Success3 = py_trees.behaviours.Success(name="Success")
    Success4 = py_trees.behaviours.Success(name="Success")
    Success5 = py_trees.behaviours.Success(name="Success")

    #TODO modulo sceneManager!
    scene_manager = SceneManagerMain("SceneManagerMain")
    """
    main_activity_scene_manager= py_trees.behaviours.SetBlackboardVariable(name="main_activity_scene_manager",
                                                        variable_name="do_speech", 
                                                        variable_value="null", 
                                                        overwrite=True)
    
    #i seguenti dummy vanno come variabili che il scene manager scrive nella blackboard. 
    dummy1 = py_trees.behaviours.SetBlackboardVariable(name="do_face",
                                                        variable_name="do_face", 
                                                        variable_value="null", 
                                                        overwrite=True)
    dummy2 = py_trees.behaviours.SetBlackboardVariable(name="do_gesture",
                                                        variable_name="do_gesture", 
                                                        variable_value="null", 
                                                        overwrite=True)
    dummy3 = py_trees.behaviours.SetBlackboardVariable(name="do_sound",
                                                        variable_name="do_sound", 
                                                        variable_value="null", 
                                                        overwrite=True)
    """
    gesture=GestureServicePytree("GestureMainActivity")
    """                                                    
    Gesture = py_trees.behaviours.Count(name="Gesture",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    #TODO sostituirlo/capire se si può usare web_service.
    Projector = py_trees.behaviours.Count(name="Projector",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    ext_speaker=SpeakerServicePytree("ExternalSpeakerMainActivity")
    """                                                  
    External_Speaker = py_trees.behaviours.Count(name="External_Speaker",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    chatbot=AWSLexServicePytree("AwsLexMainActivity")
    """
    Chat_Bot = py_trees.behaviours.Count(name="Chat_Bot",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    tts = AWSTtsServicePytree("AwsTtsMainActivity")
    """                                                  
    Tts = py_trees.behaviours.Count(name="Tts",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    stt=SpeechToTextServicePytree("SpeechToTextMainActivity")
    """                                                  
    Stt = py_trees.behaviours.Count(name="Stt",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    face_exp=FaceServicePytree("FaceMainActivity")
    """                                                  
    Facial_Expression = py_trees.behaviours.Count(name="Facial_Expression",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    speaker=SpeakerServicePytree("SpeakerMainActivity") 
    """                                                 
    Speaker = py_trees.behaviours.Count(name="Speaker",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    lips_sync=FaceServicePytree("LipsSyncMainActivity")
    """                                                  
    Lips_Synk = py_trees.behaviours.Count(name="Lips_Synk",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    microphone=MicrophoneServicePytree("MicrophoneMainActivity")
    """                                                  
    Microphone = py_trees.behaviours.Count(name="Microphone",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    #TODO mancano le foglie di imageAi 
    custom_yolo = CustomYoloServicePytree("DetectionCardMain")
    """                                                 
    Detection_Card = py_trees.behaviours.Count(name="Detection_Card",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """                                                 
    invalid_response_stt = py_trees.behaviours.SetBlackboardVariable(name="InvalidResponseMainStt",
                                                        variable_name=DetectorNameSpace.stt.name+"/result", 
                                                        variable_value="null", 
                                                        overwrite=True)
    invalid_response_card = py_trees.behaviours.SetBlackboardVariable(name="InvalidResponseMainCard",
                                                        variable_name=DetectorNameSpace.card_detect.name+"/result", 
                                                        variable_value="null", 
                                                        overwrite=True)
                                        
    counter_no_answer = CounterNoAnswer("CounterNoAnswer".
                                        variable_name= PyTreeNameSpace.invalid_response.name+"/"+PyTreeNameSpace.mainactivity.name+"/counter_no_answer") 
    #TODO timer                                                 
    timeout_kid_detection = Timer(name="TimeoutKidDetectionMain",
                                                    variable_namespace=PyTreeNameSpace.timer.name+"/"+PyTreeNameSpace.mainactivity.name, 
                                                    variable_name="kid_detection")

    parall_speaker = py_trees.composites.Parallel(name="ParallelSpeaker")
    parall_speaker.add_children([speaker,lips_sync]) 

    sequen_speaker = py_trees.composites.Sequence(name="SequenceSpeaker")
    sequen_speaker.add_children([chatbot,tts,parall_speaker])

    Either_Or_Speaker = py_trees.idioms.either_or(
        name="Either_Or_Speaker",
        conditions=[
            py_trees.common.ComparisonExpression("do_speech", "null", operator.ne),
            py_trees.common.ComparisonExpression("do_speech", "null", operator.eq),
        ],
        subtrees=[sequen_Speaker, Success1],
        namespace="either_or_speaker",
    )
    Either_Or_Face = py_trees.idioms.either_or(
        name="Either_Or_Face",
        conditions=[
            py_trees.common.ComparisonExpression("do_face", "null", operator.ne),
            py_trees.common.ComparisonExpression("do_face", "null", operator.eq),
        ],
        subtrees=[face_exp, Success5],
        namespace="either_or_face",
    )
    Either_Or_Gesture = py_trees.idioms.either_or(
        name="Either_Or_Gesture",
        conditions=[
            py_trees.common.ComparisonExpression("do_gesture", "null", operator.ne),
            py_trees.common.ComparisonExpression("do_gesture", "null", operator.eq),
        ],
        subtrees=[gesture, Success3],
        namespace="either_or_gesture",
    )
    Either_Or_External_Speaker = py_trees.idioms.either_or(
        name="Either_Or_External_Speaker",
        conditions=[
            py_trees.common.ComparisonExpression("do_sound", "null", operator.ne),
            py_trees.common.ComparisonExpression("do_sound", "null", operator.eq),
        ],
        subtrees=[ext_speaker, Success4],
        namespace="either_or_esternal_speaker",
    )

    parall_face_and_gesture = py_trees.composites.Parallel(name="ParallelFaceAndGesture")
    parall_face_and_gesture.add_children([Either_Or_Face,Either_Or_Gesture])

    sequen_speaker_and_parallel_f_g = py_trees.composites.Sequence(name="SequenceSpeakerAndParallelFG")
    sequen_speaker_and_parallel_f_g.add_children([Either_Or_Speaker,parall_Face_And_Gesture])
    
    parall_robot = py_trees.composites.Parallel(name="ParallelRobot")
    parall_robot.add_children([Either_Or_External_Speaker, sequen_speaker_and_parallel_f_g])
    
    sequen_robot = py_trees.composites.Sequence(name="SequenceRobot")
    sequen_robot.add_children([scene_manager,dummy1,dummy2,dummy3,Projector,parall_robot])

    sequen_speech_kid = py_trees.composites.Sequence(name="SequenceSpeechKid")
    sequen_speech_kid.add_children([microphone ,stt])

    parall_detect_kid = py_trees.composites.Parallel(name="ParallelDetectKid")
    parall_detect_kid.add_children([sequen_speech_kid,custom_yolo])

    Either_Or_Timer_Detection = eu.either_or(
        name="Either_Or_Timer_Detection",
        conditions=[
            py_trees.common.ComparisonExpression("timer", 10, operator.lt),
            py_trees.common.ComparisonExpression("timer", 10, operator.ge),
        ],
        preemptible = False,
        subtrees=[parall_Detect_Kid, invalid_response],
        namespace="either_or_timer_detection",
    )

    """dummydummy
    dummydummy = py_trees.behaviours.Count(name="dummydummy",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    Write_On_BB_Timer_Exceed = py_trees.behaviours.SetBlackboardVariable(name="Write_On_BB_Timer_Exceed",
                                                    variable_name="timer", 
                                                    variable_value=17, 
                                                    overwrite=True)
    dummy_sequence= py_trees.composites.Sequence(name="dummy_sequence")
    dummy_sequence.add_children([Write_On_BB_Timer,dummydummy,Write_On_BB_Timer_Exceed])
    sequen_Detect_Kid = py_trees.composites.Parallel(name="PARALLEL_Detect_Kid")
    """
    sequen_detect_kid = py_trees.composites.Sequence(name="SequenceDetectKid",memory=False)
    sequen_detect_kid.add_children([timeout_kid_detection, Either_Or_Timer_Detection])                                         

    #TODO modulo per vedere se il sottoalbero è terminato
    subtree_result = SubTreeResultMain("SubTreeMain")
    """                                            
    MainActivity_Subtree_Results = py_trees.behaviours.Count(name="Interaction_Bg_Subtree_Results",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    """
    running_or_success = rs.create_root()

    root.add_children([sequen_Robot, sequen_detect_kid, subtree_result, running_or_success])

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
    for unused_i in range(1, 12):
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