#!/usr/bin/env python

#------------------------------------------------------------------------------
# Simple Python Interface for CoRDial
# Copyright (C) 2017 Elaine Schaertl Short
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#------------------------------------------------------------------------------


import roslib; roslib.load_manifest('cordial_core')
import rospy
from cordial_core.msg import *
from std_msgs.msg import String
import actionlib

class RobotManager():
    def __init__(self,robot_name):
        
        base_topic = robot_name

        self._speech_client = actionlib.SimpleActionClient(base_topic+'/Player',PlayerAction)

        rospy.loginfo("Waiting for CoRDial Action Servers")
        rospy.loginfo(" --- Robot Behavior Controller")
        self._robot_client = actionlib.SimpleActionClient(base_topic+'/Behavior',BehaviorAction)
        self._robot_client.wait_for_server()
        rospy.loginfo(" --- Speech Player")
        self._speech_client.wait_for_server()
        rospy.loginfo("Action servers connected")


    def stop_speech(self):
        rospy.loginfo("Stopping Robot Speech")
        self._speech_client.cancel_all_goals()
        
    def say(self, phrase_name,interrupt = True, wait = False):
        rospy.loginfo("Saying: " + phrase_name)
        goal = PlayerGoal(phrase=phrase_name, interrupt=interrupt)
        self._speech_client.send_goal(goal)

        if wait:
            rospy.loginfo("Waiting for speech server result")
            self._speech_client.wait_for_result(rospy.Duration(60.0))
            if not self._speech_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                
                rospy.logwarn("Robot Manager gave up waiting for speech result. This is likely a problem. State was: "+str(self._speech_client.get_state()))
        
    def do(self, behavior_id, args = [], wait = False):
        rospy.loginfo("Sending behavior goal: " + behavior_id + " args: " + " ".join(map(lambda s: str(s), args)))
        goal = BehaviorGoal(behavior=behavior_id, if_conflict=BehaviorGoal.OVERRIDE,wait_and_block=wait, args = " ".join(map(lambda s: str(s), args)))
        self._robot_client.send_goal(goal)
        #if wait:
        rospy.loginfo("Waiting for behavior result.")
        self._robot_client.wait_for_result(rospy.Duration.from_sec(10.0))
        if not self._robot_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("Robot Manager gave up waiting for behavior result. This is likely a problem. State was: "+str(self._robot_client.get_state()))
        

    def headpose(self, r, p, y, time=0):
        self.do("headpose", args=[str(r),str(p),str(y),str(time)])
        
    def lookat(self, target, time=0):
        self.do("lookat", args=[target,str(time)])

    def lookat_xyz(self, x,y,z, time=0):
        self.do("lookat", args=[str(x),str(y),str(z),str(time)])

    def watch(self, target):
        self.do("watch", args=[target])

    def watch_off(self):
        self.do("watch_off")

def main():
    rospy.init_node("robot_manager_test")

    dm = RobotManager()

    dm.do("eye_open")
    rospy.sleep(1.0)
    #dm.load_phrases("phrases.yaml")
    dm.do("question")
    rospy.sleep(5.0)
    dm.do("surprise")
    print "waiting..."
    rospy.sleep(3.0)
    dm.do("eye_close")


    
if __name__ == '__main__':
    main()
