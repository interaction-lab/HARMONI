#!/usr/bin/env python3


PKG = "test_harmoni_pattern"
# Common Imports
import unittest, rospy, roslib, sys

# Specific Imports
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib.constants import (
    DetectorNameSpace,
    ActuatorNameSpace,
    DialogueNameSpace,
    SensorNameSpace,
    ActionType,
    State,
)
from harmoni_pytree.pytree_root import PyTreeRoot
from harmoni_common_msgs.msg import harmoniAction, harmoniFeedback, harmoniResult
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import time
import py_trees
import py_trees.console as console


class TestPyTree(unittest.TestCase):
    """This test of the sequential pattern player uses a simple mic test script.

    If the mic is launched and starts successfully (as verified by the subscriber)
    then the test is passed.
    """
    def setUp(self):
        self.feedback = State.INIT
        self.result = False
        rospy.init_node("test_pytree", log_level=rospy.INFO)
        """
        Entry point for the demo script.
        """
        self.pytree_root = PyTreeRoot()
        #args = self.pytree_root.command_line_argument_parser().parse_args()
        print(self.pytree_root.description())
        py_trees.logging.level = py_trees.logging.Level.DEBUG
        self.root = self.pytree_root.create_root()
        ####################
        # Tree Stewardship
        ####################
        print("Tree Stewardship")
        self.behaviour_tree = py_trees.trees.BehaviourTree(self.root)
        self.behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
        self.behaviour_tree.visitors.append(py_trees.visitors.SnapshotVisitor())
        additional_parameters = dict([
            (ActuatorNameSpace.tts.name,False),
            (ActuatorNameSpace.speaker.name,False),
            (ActuatorNameSpace.face.name,True),
            (DialogueNameSpace.bot.name,True)])
        self.behaviour_tree.setup(timeout=15,**additional_parameters)

        print(py_trees.display.unicode_tree(root=self.root))
        
        ####################
        # Execute
        ####################
        
        blackboardProvaIn = py_trees.blackboard.Client(name="blackboardProva", namespace=DialogueNameSpace.bot.name)
        blackboardProvaIn.register_key("result_data", access=py_trees.common.Access.WRITE)
        blackboardProvaIn.register_key("result_message", access=py_trees.common.Access.WRITE)
        blackboardProvaIn.result_message = State.SUCCESS
        blackboardProvaIn.result_data = "Vorrei ordinare dei fiori"

    def print_tree(self, tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
            

    def test_pytree(self):
        #input = self.data
        
        
        for i in range(1, 12):
            print("\n--------- Tick {0} ---------\n".format(i))
            self.behaviour_tree.tick(
                pre_tick_handler=None,
                post_tick_handler=self.print_tree
            )
            time.sleep(1.0)
            if self.behaviour_tree.root.status ==  py_trees.common.Status.SUCCESS:
                break

        rospy.loginfo("TestPyTree: Started up. waiting for pytree startup")
        return



def main():
    # TODO combine validity tests into test suite so that setup doesn't have to run over and over.
    import rostest

    rospy.loginfo("test_pytree started")
    rospy.loginfo("TestPyTree: sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, "test_pytree", TestPyTree, sys.argv)


if __name__ == "__main__":
    main()
