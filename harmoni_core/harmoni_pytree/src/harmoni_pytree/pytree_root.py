#!/usr/bin/env python3

##############################################################################
# Imports
##############################################################################
import rospy
from harmoni_pytree.aws_tts_service_pytree import AWSTtsServicePytree
from harmoni_pytree.aws_lex_service_pytree import AWSLexServicePytree
from harmoni_pytree.speaker_service_pytree import SpeakerServicePytree
from harmoni_pytree.face_service_pytree import FaceServicePytree
from harmoni_common_lib.constants import ActuatorNameSpace, DialogueNameSpace, State
import argparse
import py_trees
import sys
import time

import py_trees.console as console

##############################################################################
# Classes
##############################################################################

class PyTreeRoot:
    def description(self):
        content = "Demonstrates sequences in action.\n\n"
        content += "A sequence is populated with 2-tick jobs that are allowed to run through to\n"
        content += "completion.\n"

        if py_trees.console.has_colours:
            banner_line = console.green + "*" * 79 + "\n" + console.reset
            s = "\n"
            s += banner_line
            s += console.bold_white + "Sequences".center(79) + "\n" + console.reset
            s += banner_line
            s += "\n"
            s += content
            s += "\n"
            s += banner_line
        else:
            s = content
        return s


    def epilog(self):
        if py_trees.console.has_colours:
            return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
        else:
            return None


    def command_line_argument_parser(self):
        parser = argparse.ArgumentParser(description=self.description,
                                        epilog=self.epilog,
                                        formatter_class=argparse.RawDescriptionHelpFormatter,
                                        )
        parser.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
        return parser


    def create_root(self):
        root = py_trees.composites.Sequence("Sequence")
        tts = AWSTtsServicePytree("AwsTtsPyTreeTest")
        chatbot = AWSLexServicePytree("AwsLexPyTreeTest")
        speaker = SpeakerServicePytree("SpeakerPyTreeTest")
        face = FaceServicePytree("FacePyTreeTest")
        parall_speaker_face = py_trees.composites.Parallel("Parallel")
        root.add_child(chatbot)
        root.add_child(tts)
        root.add_child(parall_speaker_face)
        parall_speaker_face.add_child(speaker)
        parall_speaker_face.add_child(face)
        return root


