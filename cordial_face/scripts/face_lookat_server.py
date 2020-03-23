#!/usr/bin/python

#------------------------------------------------------------------------------
# Interface between the SPRITE face and CoRDial, for looking at locations
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


import roslib; roslib.load_manifest('cordial_face')
import rospy
import tf
from cordial_face.msg import *
import threading
from geometry_msgs.msg import Point
import sys

class LookatServer:
    def __init__(self,robot_name):
        base_topic = "/CoRDial/"+robot_name+"/"
        self._base_topic=base+_topic
        self._face_pub = rospy.Publisher(base_topic+'face', FaceRequest)
        self._lookat_sub = rospy.Subscriber(base_topic+'lookat', LookatRequest, self.lookat_cb)

        self._shared = {"target": "", "tracking":False}
        self._tf = tf.TransformListener()
        self._track_thread = threading.Thread(target=self.tracking_thread)
        self._track_thread.start()

    def tracking_thread(self):
        tf_rate = 20#hz
        r = rospy.Rate(tf_rate)

        while not rospy.is_shutdown():
            if self._shared["tracking"]:
                target = self._shared["target"]
                time = rospy.Time.now()
                try:
                    (trans,rot) = self._tf.lookupTransform(self._base_topic+'lookat_frame',target, rospy.Time(0))
                    f = FaceRequest(hold_gaze=FaceRequest.IDLE_OFF, retarget_gaze=True, gaze_target=Point(x=trans[0]*100,y=trans[1]*100,z=trans[2]*100)) #face deals in cm, tf in m
                    self._face_pub.publish(f)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("Lookat server can't transform from frame " + target + " to DragonBot/lookat_frame.")
            r.sleep()

    def lookat_cb(self, goal):
        if goal.follow_frame:
            self._shared["target"] = goal.frameid
            self._shared["tracking"] = True
        else:
            self._shared["tracking"] = False
            self._shared["target"] = ""



if __name__=='__main__':
    rospy.init_node('cordial_lookat')
    l = LookatServer(sys.argv[1])
    while not rospy.is_shutdown():
        rospy.spin()
