#!/usr/bin/env python3
##############################################################################
# Imports
##############################################################################

import argparse
import functools
import py_trees
import rospy
import time
import subprocess
import py_trees.console as console
import os
from harmoni_common_lib.constants import *
from harmoni_pytree.leaves.yolo_service import ImageAIYoloServicePytree
from harmoni_pytree.subtrees import session as s
from harmoni_pytree.subtrees import reset as r
from harmoni_pytree.subtrees import on_modules as o 

##############################################################################
# Classes
##############################################################################
"""
Root
"""
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

    root = py_trees.composites.Selector(name="root")

    on_module = o.create_root()

    yolo_service = ImageAIYoloServicePytree("FaceDetection")

    session = s.create_root()

    parallel_onModule_detectFace_session = py_trees.composites.Parallel(name="ParallelOnModuleDetectFaceSession")
    
    parallel_onModule_detectFace_session.add_children([on_module, yolo_service, session])
    
    reset = r.create_root()

    root.add_children([parallel_onModule_detectFace_session, reset, py_trees.behaviours.Running()])

    """
    b1 = root.attach_blackboard_client(name="b1", namespace=DetectorNameSpace.card_detect.name)
    b1.register_key("result", access=py_trees.common.Access.WRITE)
    b1.result = "null"

    b2 = root.attach_blackboard_client(name="b2", namespace= PyTreeNameSpace.visual.name)
    b2.register_key("finished", access=py_trees.common.Access.WRITE)
    b2.finished = False
    """
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

    rospy.init_node("root_default", log_level=rospy.INFO)

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
    #while True:
    for unused_i in range(1, 70):
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