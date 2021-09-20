#!/usr/bin/env python3
##############################################################################
# Imports
##############################################################################

import argparse
import functools
from py_trees.blackboard import Blackboard
from py_trees.idioms import either_or
import py_trees
import time
import subprocess
import py_trees.console as console
import os
import session
import reset

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
        s += console.bold_white + "Root".center(79) + "\n" + console.reset
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
    root = py_trees.composites.Selector(name="root",memory=True)

    Session = session.create_root()
    
    Reset = reset.create_root()

    root.add_children([Session, Reset, py_trees.behaviours.Running()])

    b1 = root.attach_blackboard_client(name="b1", namespace="microphone_bb_namespace")
    b2 = root.attach_blackboard_client(name="b2", namespace="scene/stt_bb_namespace")
    b3 = root.attach_blackboard_client(name="b3", namespace="bot_output_bb_namespace")
    b4 = root.attach_blackboard_client(name="b4", namespace="tts_bb_namespace")
    b5 = root.attach_blackboard_client(name="b5", namespace="scene/detection_card_bb_namespace")
    b6 = root.attach_blackboard_client(name="b6", namespace="scene/detection_face_bb_namespace")
    b7 = root.attach_blackboard_client(name="b7", namespace="scene/gesture_bb_namespace")
    b8 = root.attach_blackboard_client(name="b8", namespace="scene/face_bb_namespace")
    b9 = root.attach_blackboard_client(name="b9", namespace="scene/projector_bb_namespace")
    b10 = root.attach_blackboard_client(name="b10", namespace="scene/chatbot_in_namespace")
    b11 = root.attach_blackboard_client(name="b11", namespace="scene/external_speaker_in_namespace")
    b12 = root.attach_blackboard_client(name="b11", namespace="scene/counter_no_answer")

    b10.register_key("result_data", access=py_trees.common.Access.WRITE)
    b10.register_key("result_message", access=py_trees.common.Access.WRITE)
    b8.register_key("result_data", access=py_trees.common.Access.WRITE)
    b8.register_key("result_message", access=py_trees.common.Access.WRITE)
    b6.register_key("result_data", access=py_trees.common.Access.WRITE)
    b6.register_key("result_message", access=py_trees.common.Access.WRITE)
    b1.register_key("result_data", access=py_trees.common.Access.WRITE)
    b1.register_key("result_message", access=py_trees.common.Access.WRITE)

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
        target_directory = os.path.join(os.getcwd(), "dot_folder/")
        py_trees.display.render_dot_tree(root = root,target_directory = target_directory)
        if py_trees.utilities.which("xdot"):
            try:
                subprocess.call(["xdot", target_directory+"%s.dot" % root.name])
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