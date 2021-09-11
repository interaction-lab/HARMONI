##############################################################################
# Imports
##############################################################################

import argparse
import functools
from os import name
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
        s += console.bold_white + "Non ti vedo".center(79) + "\n" + console.reset
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


def create_root(name = "Non_Ti_Vedo"):
    root = py_trees.composites.Sequence(name = name)
    
    Success = py_trees.behaviours.Success(name="Success")

    Scene_Manager_Non_Ti_Vedo = py_trees.behaviours.SetBlackboardVariable(name="Scene_Manager_Non_Ti_Vedo(do_speech)",
                                                        variable_name="scene/chatbot_in_namespace/result_data", 
                                                        variable_value="non vedo il bimbo (intent)", 
                                                        overwrite=True)
    dummy1 = py_trees.behaviours.SetBlackboardVariable(name="do_face",
                                                        variable_name="scene/face_bb_namespace/result_data", 
                                                        variable_value="Triste", 
                                                        overwrite=True)
    Chat_Bot = py_trees.behaviours.SetBlackboardVariable(name="Chat_bot",
                                                        variable_name="bot_output_bb_namespace/result_data", 
                                                        variable_value="Non ti vedo, dove sei?", 
                                                        overwrite=True)
    Tts = py_trees.behaviours.Count(name="Tts",
                                        fail_until=0,
                                        running_until=1,
                                        success_until=10,
                                        reset=False)
    Stt = py_trees.behaviours.Count(name="Stt",
                                        fail_until=0,
                                        running_until=1,
                                        success_until=10,
                                        reset=False)
    Facial_Expression = py_trees.behaviours.Count(name="Facial_Expression",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    Speaker = py_trees.behaviours.Count(name="Speaker",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    Lips_Synk = py_trees.behaviours.Count(name="Lips_Synk",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    Microphone = py_trees.behaviours.Count(name="Microphone",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    Detection_Face = py_trees.behaviours.SetBlackboardVariable(name="Detection_Face",
                                                        variable_name="scene/detection_face_bb_namespace/result_message", 
                                                        variable_value="non null", 
                                                        overwrite=True)
    Invalid_BB_Speech = py_trees.behaviours.Count(name="Invalid_BB_Speech",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)
    Write_On_BB_Timer = py_trees.behaviours.SetBlackboardVariable(name="Write_On_BB_Timer",
                                                    variable_name="timer", 
                                                    variable_value=5, 
                                                    overwrite=True)
    Subtree_Results = py_trees.behaviours.Count(name="Subtree_Results",
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10,
                                                      reset=False)

    parall_Speaker = py_trees.composites.Parallel(name="Parallel_Speaker")
    parall_Speaker.add_children([Speaker,Lips_Synk])    

    sequen_Speech_Kid = py_trees.composites.Sequence(name="Sequence_Speech_Kid")
    sequen_Speech_Kid.add_children([Microphone ,Stt])

    Either_Or_Timer_Detection = eu.either_or(
        name="Either_Or_Timer_Detection",
        conditions=[
            py_trees.common.ComparisonExpression("timer", 10, operator.lt),
            py_trees.common.ComparisonExpression("timer", 10, operator.ge),
        ],
        preemptible = False,
        subtrees=[sequen_Speech_Kid, Invalid_BB_Speech],
        namespace="either_or_timer_detection",
    )

    sequen_Detect_Kid = py_trees.composites.Sequence(name="Sequence_Detect_Kid",memory=False)
    sequen_Detect_Kid.add_children([Write_On_BB_Timer, Either_Or_Timer_Detection])                                         

    parall_Detect_And_Face = py_trees.composites.Parallel(name="Parallel_Detect_And_Face")
    parall_Detect_And_Face.add_children([sequen_Detect_Kid, Facial_Expression])  

    sequen_Non_Ti_Vedo = py_trees.composites.Sequence(name="Sequence_Non_Ti_Vedo")
    sequen_Non_Ti_Vedo.add_children([Scene_Manager_Non_Ti_Vedo,dummy1,Chat_Bot,Tts,parall_Speaker,parall_Detect_And_Face])

    Either_Or_Non_Ti_Vedo = eu.either_or(
        name="Either_Or_Non_Ti_Vedo",
        conditions=[
            py_trees.common.ComparisonExpression("bb_face_detection", "null", operator.ne),
            py_trees.common.ComparisonExpression("bb_face_detection", "null", operator.eq),
        ],
        preemptible = False,
        subtrees=[Success, sequen_Non_Ti_Vedo],
        namespace="either_or_non_ti_vedo",
    )
    Running_Or_Success = rs.create_root()

    root.add_children([Detection_Face,Either_Or_Non_Ti_Vedo, Subtree_Results, Running_Or_Success])

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