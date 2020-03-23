#!/usr/bin/env python

#------------------------------------------------------------------------------
# Interface between SPRITE control nodes and CoRDial
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


import roslib; roslib.load_manifest('cordial_sprite')
import rospy
import actionlib
import sys
from cordial_core.msg import *
from cordial_face.msg import FaceRequest
from cordial_sprite.msg import *

class DragonbotCoRDialServer():
    _feedback = BehaviorFeedback()
    _result = BehaviorResult()

    def __init__(self):
        self._visemes = ['AO_AW', 'CH_SH_ZH', 'R_ER', 'L', 'IDLE', 'AA_AH', 'EY', 'M_B_P', 'N_NG_D_Z', 'EH_AE_AY', 'OO']

        base_topic = "" 

        self._server = actionlib.SimpleActionServer(base_topic+"Behavior", BehaviorAction, execute_cb=self.execute, auto_start=False)


        self._face_pub = rospy.Publisher(base_topic+'face', FaceRequest, queue_size=10)
        rospy.sleep(0.5)

        self._last_viseme=rospy.Time.now()

        self._keyframe_client = actionlib.SimpleActionClient(base_topic+'KeyframePlayer', KeyframePlayerAction)
        self._keyframe_client.wait_for_server()

        self._server.start()
        rospy.loginfo("SPRITE CoRDial server started.")
        

    def play_viseme(self, viseme_name):
        viseme_speed = 10#ms
        req = FaceRequest(visemes=[viseme_name], viseme_ms=viseme_speed, times=[0])
        self._face_pub.publish(req)

    def play_expression(expression_name):
        pass

    def execute(self, goal):
        rospy.loginfo("SPRITE CoRDial Server got goal: " + goal.behavior)
        success=True
        if self._server.is_preempt_requested():
            rospy.loginfo("SPRITE CoRDial server preempted")
            self._server.set_preempted()
            success = False

        viseme_spacing=0.1
        if goal.behavior in self._visemes:
            pass
            '''
            elapsed = rospy.Time.now()-self._last_viseme
            rospy.loginfo(str(elapsed.to_sec()))
            if elapsed>rospy.Duration.from_sec(viseme_spacing):
                self._last_viseme = rospy.Time.now()
                self.play_viseme(goal.behavior)
            else:
                rospy.loginfo("Dropping viseme: " + goal.behavior)
            '''
        else:            
            bg = KeyframePlayerGoal(behavior=goal.behavior, args=goal.args)
            self._keyframe_client.send_goal(bg)

        feedback = BehaviorFeedback(status=BehaviorFeedback.PLAYING)
        self._server.publish_feedback(feedback)

        self._keyframe_client.wait_for_result()
        if success:
            result = BehaviorResult(result=BehaviorResult.DONE)
            self._server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node("cordial_sprite")
    DragonbotCoRDialServer()
    while not rospy.is_shutdown():
        rospy.spin()
