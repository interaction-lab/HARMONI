#!/usr/bin/env python3

import rospy
import roslib
from harmoni_common_lib import constants
from harmoni_common_msgs.msg import Object2D, Object2DArray

from behavior_pattern import BehaviorPattern


class FaceTracker(BehaviorPattern):
    """Track Faces

        Args:
            track_threshold(tuple: (int,int)): x,y # pixels to ignore from each side of image center #TODO change this to a percentage instead
    """
    def __init__(self, track_threshold = (50,50)):

        self._face_sub = None
        self._pause = False
        self._track_threshold = track_threshold
        super.__init__()
        return

    def start(self):
        self._face_sub = rospy.Subscriber(constants.RouterDetector.face_detect.value, Object2DArray, self.detect_callback)
        super().start()
        if (self._face_sub != None):
            self.update(State.START)
            rospy.loginfo("Face tracker started.")
        else:
            self.update(State.FAILED)
            rospy.logerr("Face tracker failed to start.")
        return
       
    def stop(self):
        self.update(State.SUCCESS)
        try:
            self._image_sub.unregister()
            rospy.loginfo("Face tracker stopped.")
        except rospy.ROSInternalException:
            rospy.logwarn("Received stop call on face tracker that was already stopped.")
        return
    
    def pause(self):
        self._pause = True
        rospy.loginfo("Face tracker paused.")
        return

    
    def detect_callback(self,data):
        #start outputting actuator directions for head motor and face/eye movement
        #TODO send commands to actuators instead of just printing desired behavior
        #TODO handle multiple faces instead of just first one.
        face = data.objects[0]
        if face == None:
            print("Face Tracker: No face to track.")
        else if isinstance(face, Object2D):
            img_w, img_h, x, y = face.width, face.height, face.center_x, face.center_y
            img_center_x = img_w / 2
            img_center_y = img_h / 2
            if x > img_center_x + self._track_threshold[0]:
                print("Face Tracker: Face on right, move head right.")
            else if x < img_center_x - self._track_threshold[0]:
                print("Face Tracker: Face on left, move head left.")
            if y > img_center_y + self._track_threshold[1]:
                print("Face Tracker: Face above, move head up.")
            else if y < img_center_y - self._track_threshold[1]:
                print("Face Tracker: Face below, move head down.")
        else:
            rospy.logerr("Face Tracker received bad face object from detector.")


         

if __name__ == "__main__":
    BehaviorPattern.launch(constants.BehaviorPattern.tracking.name, FaceTracker())