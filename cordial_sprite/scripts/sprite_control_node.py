#!/usr/bin/python


#------------------------------------------------------------------------------
# {one line to give the program's name and a brief idea of what it does.}
# Copyright (C) 2017 Kara Douville and Elaine Schaertl Short
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


#Pieces of code were taking from both websites below to help write
#the functions that read from and write to the servos
#http://martinsant.net/?page_id=402
#http://spinlock.blogspot.fr/2012/02/simple-servo-actuation-in-ros.html

import roslib; roslib.load_manifest('cordial_sprite')
import json
import rospy
import actionlib
from cordial_sprite.msg import *
from cordial_face.msg import *
from hexapod_ik import Hexapod
import serial, math, time, struct, datetime
from interpolation import Interpolation
from MicroMaestro6Channel import MotorController
import threading
import tf
import math
import sys
import ast
import argparse
import random

DELTA_T = 0.1

class SPRITEAnimator:
   # create messages that are used to publish feedback/result
   _feedback = cordial_sprite.msg.KeyframePlayerFeedback()
   _result   = cordial_sprite.msg.KeyframePlayerResult()

   def __init__(self, filename, port, robot_name, zeros):
      #start action server

      self._base_topic = ""
      base_topic=self._base_topic

      self._robot_name = robot_name

      self._server = actionlib.SimpleActionServer(base_topic+"KeyframePlayer", KeyframePlayerAction, execute_cb=self.execute_cb, auto_start = False)
      self._face_pub = rospy.Publisher(base_topic+'face_keyframes', FaceKeyframeRequest, queue_size=10)
      self._face_lookat = rospy.Publisher(base_topic+'lookat', LookatRequest, queue_size=10)
      rospy.sleep(0.5)

      rospy.loginfo("Starting motor controller...")
      self.mc = MotorController(zeros,port)
      self.mc.set_a_all(0)
      rospy.loginfo("Success!")

      #read in behaviors and keyframes from json file to dictonary
      rospy.loginfo("Loading keyframes from file...")
      with open(filename,'r') as data_file:
         data = json.load(data_file)
      self.KF_behavior_dict = data
      rospy.loginfo("Done loading keyframes.")

      self.i = Interpolation()

      self._current_pose = [0,0,0,0,0,0]
      self.move_robot([0,0,0,0,0,0], vlim=5)
      rospy.sleep(3.0)

      self._thread_dict= {"lookat":False, "idle":False}

      self._tb = tf.TransformBroadcaster()
      self._tl = tf.TransformListener()
      self._tf_thread = threading.Thread(target=self.pose_pub)
      self._tf_thread.start()

      #self._idle_thread = threading.Thread(target=self.idle)
      #self._idle_thread.start()


      self._thread_dict["idle"]=True
      self._thread_dict["preempt"]=False
      self._thread_dict["moving"]=True
      self._server.start()
      rospy.loginfo("Keyframe player server started.")

   def idle(self):
      current_pose= self._current_pose
      dofs = ['z','pa','ya']
      dof_i = [2,4,5]
      movements = [1,10,10]
      last_direction = [1,1,1]

      while not rospy.is_shutdown():
         if self._thread_dict["idle"]==False:
            rospy.sleep(0.1)
         else:
            i = random.choice(range(len(dofs)))

            target_dofs = [dofs[i]]
            target_positions = [[current_pose[dof_i[i]]+last_direction[i]*movements[i]]]
            last_direction[i]*=-1

            self.move_through_frames(target_positions,[1.0],dofs=target_dofs)
            rospy.sleep(random.randrange(50,400)/100)



   def pose_pub(self):
      tf_rate = 1/DELTA_T#hz
      r = rospy.Rate(tf_rate)
      while not rospy.is_shutdown():
         pose = self._current_pose
         self._tb.sendTransform((pose[0]/100, pose[1]/100, pose[2]/100),
                                tf.transformations.quaternion_from_euler(math.radians(pose[3]), math.radians(-pose[4]),math.radians(-pose[5])),
                                rospy.Time.now(),
                                "CoRDial/"+self._robot_name+"/platform_center",
                                "CoRDial/"+self._robot_name+"/platform_zero")
         r.sleep()

   def move_robot(self, pose_6d, vlim = 0, alim = 0):
      rospy.loginfo("Moving to frame: " + str(pose_6d))
      self.mc.set_a_all(alim)
      self.mc.set_v_all(vlim)

      frame = self.config_space(pose_6d)

      for m in range(6):
         if m % 2 == 1:
            frame[m]=-frame[m]

      for m in range(6):
         if self.mc.set_motor_angle(m, frame[m]):
            self._current_pose = pose_6d


   def lookat_point(self, pose3d, time=0):
      target = [i for i in self._current_pose]
      p,y = self.dir_to_point(pose3d, target)
      target[4] = p
      target[5] = -y
      keyframes=[target]
      times = [time]
      return self.move_through_frames(keyframes,times)

   def dir_to_point(self, pose3d, origin=[0,0,0]):
      x = pose3d[0]-origin[0]
      y = pose3d[1]-origin[1]
      z = pose3d[2]-origin[2]
      yaw = math.degrees(math.atan2(y,x))
      pitch = math.degrees(math.atan2(z, math.sqrt(y*y+x*x)))
      return pitch,yaw

   def lookat_frame(self, frameid, time = 0, vlim = 0, alim = 0):
      try:
         (trans,rot) = self._tl.lookupTransform("CoRDial/"+self._robot_name+'/platform_zero', frameid, rospy.Time(0))
         trans =  map(lambda x: x*100, trans)
         return self.lookat_point(trans)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
         rospy.logwarn("Lookat server can't transform from frame " + frameid + " to " + "CoRDial/"+self._robot_name+'/platform_zero')
         return False

   def track_frame(self,frameid):
      rate = rospy.Rate(1)#Hz
      l = LookatRequest(follow_frame=True, frameid=frameid)
      self._face_lookat.publish(l)
      stop_lookat = False
      while not rospy.is_shutdown() and not self._server.is_preempt_requested() and not stop_lookat:
         self.lookat_frame(frameid)
         rate.sleep()
         stop_lookat=not self._thread_dict["lookat"]
         success = True
      if self._server.is_preempt_requested() or stop_lookat:
         l = LookatRequest(follow_frame=False)
         self._face_lookat.publish(l)
         rospy.loginfo('SPRITEAnimator: Preempted')
         self._server.set_preempted()
         if stop_lookat:
            success = True
         else:
            success = False
      return success

   def stop_tracking(self):
      self._thread_dict["lookat"]=False


   def adjust_timing(self, keyframes, times, max_v = 50): #max_v in deg/s
      for i in range(len(keyframes)-1):
         start = keyframes[i]
         target = keyframes[i+1]

         motors_s = self.config_space(start)
         motors_t = self.config_space(target)

         d = [0,0,0,0,0,0]

         for j in range(6):
            d[j] = abs(motors_t[j]-motors_s[j])

         min_t = max(map(lambda dist: dist/max_v, d))

         if times[i+1]-times[i] < min_t:
            time_adj = min_t-(times[i+1]-times[i])
            rospy.logwarn("Moving too quickly; adjusting frames after frame " + str(i) + " by " + str(time_adj) + " seconds")
            for j in range(i+1,len(times)):
               times[j] = times[j] + time_adj
      return keyframes, times


   def time_adjusted_spline(self, keyframes, times, max_dx = 60, max_dv = 30):
      rospy.loginfo("Adjusting spline")
      spline = self.i.interpolate(keyframes,times)
      dt = 0.33 # 3Hz for initial check
      poses = self.i.get_poses(spline, dt)
      pre = poses[1:]
      post = poses[:-1]


   def broadcast_move_start(self):
      self._thread_dict["moving"]=True

   def broadcast_move_end(self):
      self._thread_dict["moving"]=False

   def catch_preemption(self):
      self._thread_dict["preempt"]=False

   def preempt_movement(self):
      if self._thread_dict["moving"]==True:
         self._thread_dict["preempt"]=True
         while self._thread_dict["preempt"]:
            rospy.sleep(0.05)
         rospy.sleep(0.1)

   def move_through_frames(self,keyframes,times, dofs = ["x","y","z","ra","pa","ya"]):
      self.broadcast_move_start()
      success = True
      r = rospy.Rate(1 / DELTA_T)

      # move to first pose TODO: LIMIT BY MOTOR SPEED
      start = self._current_pose

      all_dofs = ["x","y","z","ra","pa","ya"]
      def fill_in_dofs(frame):
         outframe = [n for n in start]
         for i in range(len(dofs)):
            outframe[all_dofs.index(dofs[i])]=frame[i]
         return outframe

      keyframes= map(lambda f: fill_in_dofs(f), keyframes)

      keyframes = [start]+keyframes
      times = [0]+times

      if len(keyframes) == 2:
         check = False
         for i in range(6):
            check = check or not abs(keyframes[1][i] - keyframes[0][i])<.01
         if not check:
            rospy.logwarn("No movement needed; already at target")
            self.broadcast_move_end()
            return success

      keyframes,times = self.adjust_timing(keyframes,times)

      try:
         spline = self.i.interpolate(keyframes, times)
      except SystemError:
         rospy.logerr("Cannot interpolate; check your keyframes!")
         return False
      poses = self.i.get_poses(spline, DELTA_T)

      if self._server.is_preempt_requested() or self._thread_dict["preempt"]==True:
         rospy.loginfo('SPRITEAnimator: Preempted')
         self.catch_preemption()
         if self._server.is_preempt_requested():
            self._server.set_preempted()
         success = False
         #TODO: clean up?
      else:
         #if no preempt, run the requested behavior goal
         start = rospy.Time.now()
         for i in range(len(poses)):
            if rospy.Time.now()-start>rospy.Duration(DELTA_T*i):
               continue
            frame = poses[i]
            if self._server.is_preempt_requested() or self._thread_dict["preempt"]==True:
               rospy.loginfo('SPRITEAnimator: Preempted')
               self.catch_preemption()
               success = False
               break
            else:
               #self.move_robot(frame)
               self.move_robot_timed(frame,DELTA_T)
            #TODO: publish TF
            r.sleep()
      self.broadcast_move_end()
      return success


   def move_robot_timed(self,pose_6d,time):
      rospy.loginfo("Moving to frame: " + str(pose_6d) + " in time " + str(time))
      #self.mc.set_a_all(alim)
      #self.mc.set_v_all(vlim)

      frame = self.config_space(pose_6d)
      cur_frame=self.config_space(self._current_pose)
      #print "frame: " + str(cur_frame)

      for m in range(6):
         if m % 2 == 1:
            frame[m]=-frame[m]
            cur_frame[m]=-cur_frame[m]

      for m in range(6):
         speed = abs(cur_frame[m]-frame[m])/time
         #print cur_frame[m]-frame[m]
         #print speed
         self.mc.set_speed(m,speed)

         if self.mc.set_motor_angle(m, frame[m]):
            self._current_pose = pose_6d

   def execute_cb(self, goal):
      rospy.loginfo("SPRITE Animator Server got goal: " + str(goal))
      self._thread_dict["idle"]=False
      #self.preempt_movement() #only necessary if using idle
      success=True

      if goal.behavior == "lookat":
         l = LookatRequest(follow_frame=True, frameid=goal.args[0])
         self._face_lookat.publish(l)

         time = 0
         if len(goal.args)==1 or len(goal.args)==2:
            if len(goal.args)==2:
               time= float(goal.args[1])
            success = self.lookat_frame(goal.args[0],time=time)
         elif len(goal.args)==4 or len(goal.args)==3:
            if len(goal.args)==4:
               time = float(goal.args[3])
            success = self.lookat_point([float(goal.args[0]),float(goal.args[1]),float(goal.args[2])], time=time)
         else:
            rospy.loginfo("Wrong number of arguments for lookat. Got: " + str(goal.args))
            success=False
         rospy.sleep(0.5)
         l = LookatRequest(follow_frame=False, frameid=goal.args[0])
         self._face_lookat.publish(l)
      elif goal.behavior=="watch":
         if len(goal.args)==1:
            success = self.track_fame(goal.args[0])
         else:
            rospy.loginfo("Wrong number of arguments for watch. Got: " + str(goal.args))
            success=False
      elif goal.behavior=="watch_off":
         self.stop_tracking()
      elif goal.behavior=="headpose":
         if len(goal.args)==3 or len(goal.args)==4:
            time = 0
            if len(goal.args)==4:
               time = float(goal.args[3])
            success = self.move_through_frames([map(lambda s: float(s),goal.args)],[time], dofs = ["ra","pa","ya"])
         else:
            rospy.loginfo("Wrong number of arguments for watch. Got: " + str(goal.args))
            success=False
      else:
         success=self.play_behavior(goal)
      #publish feedback of requested behavior
      if success:
         _feedback = goal.behavior
         self._server.publish_feedback(self._feedback)
         rospy.loginfo('SPRITEAnimator: Succeeded')
         self._server.set_succeeded(self._result)
      elif self._server.is_preempt_requested():
         rospy.loginfo('SPRITEAnimator: Preempted')
         self._server.set_preempted()
      else:
         rospy.loginfo('SPRITEAnimator: Aborted')
         self._server.set_aborted()
      self._thread_dict["idle"]=True


   def play_behavior(self, goal):
      success = True
      behavior = goal.behavior
      if not behavior in self.KF_behavior_dict.keys():
         rospy.logwarn("Invalid behavior: " + behavior + ", ignoring.")
         success = False
      else:
         frames = self.KF_behavior_dict[behavior]["keyframes"]

         keyframes = map(lambda f: f["pose"], frames)
         times = map(lambda f: float(f["time"]), frames)

         dofs = map(lambda s: str(s), self.KF_behavior_dict[behavior]["dofs"])


         body_dofs = ["x","y","z","ra","pa","ya"]
         body_indices = map(lambda d: dofs.index(d), filter(lambda s: s in dofs, body_dofs))
         body_frames = map(lambda k: [k[j] for j in body_indices], keyframes)
         body_dofs = [dofs[i] for i in body_indices]

         face_indices = range(len(dofs))
         for i in body_indices:
            face_indices.remove(i)

         if len(face_indices)>0:
            face_frames = map(lambda k: [k[j] for j in face_indices], keyframes)
            face_frames = map(lambda l: Keyframe(positions=l), face_frames)
            face_dofs = [dofs[i] for i in face_indices]
            face_req = FaceKeyframeRequest(face_dofs=face_dofs, times = times, frames=face_frames)
            self._face_pub.publish(face_req)


         success = self.move_through_frames(body_frames, times, body_dofs)
      return success


   #takes a 1D array of 6 elements (x,x,y,u,v,w) and converts to hexapod terms
   def config_space(self, pose):
      x = -pose[0]
      y = pose[1]
      z = pose[2]
      u = pose[3]
      v = pose[4]
      w = pose[5]
      h = Hexapod()
      c = list(h.best_effort_ik(x,y,z,u,v,w))
      return c

   #TODO - function needs
   def check_vel(x1, x2, dt):
      pass

if __name__ == '__main__':
   parser = argparse.ArgumentParser(description='Controller for the movement of the SPRITE robot')
   parser.add_argument('-p', '--port', help='Serial port robot is attached on')
   parser.add_argument('-n', '--name', help='What name to use for the robot?')
   parser.add_argument('-b', '--behavior-file', help="Path to the json file containing keyframes for robot behaviors")
   parser.add_argument('-z', '--zeros', nargs=6, help="Zero positions (in ticks) for the 6 motors on the SPRITE, separated by spaces")

   args = parser.parse_known_args()[0]

   rospy.init_node('sprite_animator')
   port = args.port
   name = args.name
   behav_file = args.behavior_file
   zeros=map(lambda s: int(s), args.zeros)

   da = SPRITEAnimator(behav_file, port, name,zeros)

   while not rospy.is_shutdown():
      rospy.spin()
