#!/usr/bin/python

# ------------------------------------------------------------------------------
# Interface between the SPRITE face and CoRDial, for keyframed behaviors
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
# ------------------------------------------------------------------------------

import sys
from geometry_msgs.msg import Point
import threading
from cordial_face.msg import *
import tf
import rospy
import roslib
roslib.load_manifest('cordial_face')


class LookatServer:
    def __init__(self, robot_name):
        base_topic = ""
        self._base_topic = base_topic
        self._robot_name = robot_name
        self._face_pub = rospy.Publisher(base_topic + 'face', FaceRequest, queue_size=10)
        self._lookat_sub = rospy.Subscriber(base_topic + 'lookat', LookatRequest, self.lookat_cb)
        self._keyframe_sub = rospy.Subscriber(base_topic + 'face_keyframes', FaceKeyframeRequest, self.keyframe_cb)

        self._shared = {"target": "", "tracking": False, "stop_keyframes": False}
        self._tf = tf.TransformListener()
        self._track_thread = threading.Thread(target=self.tracking_thread)
        self._track_thread.start()
        rospy.loginfo("Face ROS server started.")

    def tracking_thread(self):
        tf_rate = 5  # hz
        r = rospy.Rate(tf_rate)

        while not rospy.is_shutdown():
            if self._shared["tracking"]:
                target = self._shared["target"]
                time = rospy.Time.now()
                try:
                    (trans, rot) = self._tf.lookupTransform("/CoRDial/" + self._robot_name + '/lookat_frame', target, rospy.Time(0))
                    f = FaceRequest(hold_gaze=FaceRequest.IDLE_OFF, retarget_gaze=True, gaze_target=Point(x=trans[0] * 100, y=trans[1] * 100, z=trans[2] * 100))  # face deals in cm, tf in m
                    self._face_pub.publish(f)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Lookat server can't transform from frame " + target + " to " + self._robot_name + '/lookat_frame')
            r.sleep()

    def lookat_cb(self, goal):
        if goal.follow_frame:
            self._shared["target"] = goal.frameid
            self._shared["tracking"] = True
        else:
            self._shared["tracking"] = False
            self._shared["target"] = ""

    # TODO: make playing face keyframes interruptable (action?)
    def keyframe_cb(self, goal):
        aus = map(lambda s: s[2:], goal.face_dofs)
        elapsed = 0
        for i in range(len(goal.frames)):
            poses = goal.frames[i].positions
            time = int(goal.times[i] * 1000) - elapsed
            req = FaceRequest(aus=aus, au_degrees=poses, au_ms=time)
            self._face_pub.publish(req)
            rospy.sleep(time / 1000.0)
            elapsed += time


if __name__ == '__main__':
    rospy.init_node('cordial_lookat')
    l = LookatServer(sys.argv[1])
    while not rospy.is_shutdown():
        rospy.spin()
