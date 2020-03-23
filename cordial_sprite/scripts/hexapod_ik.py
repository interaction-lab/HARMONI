#!/usr/bin/env python

#------------------------------------------------------------------------------
# Python implementation of rotary stewart platform code from https://github.com/MarginallyClever/RotaryStewartPlatform
# Copyright (C) 2013 Dan Royer (dan@marginallyclever.com)
# Permission is hereby granted, free of charge, to any person obtaining a 
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation 
# the rights to use, copy, modify, merge, publish, distribute, sublicense, 
# and/or sell copies of the Software, and to permit persons to whom the 
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#------------------------------------------------------------------------------



#functions to do IK on a hexapod
from math import *
import copy

#measurements based on computer model of robot
BICEP_LENGTH = 5.000
FOREARM_LENGTH =16.750
SWITCH_ANGLE = 19.690
#top center to wrist hole: X7.635 Y+/-0.553 Z0.87
T2W_X = 7.635
T2W_Y = 0.553
T2W_Z = -0.870
#base center to shoulder hole: X8.093 Y+/-2.15 Z7.831
B2S_X = 8.093
B2S_Y = 2.150
B2S_Z = 6.618

#some experimentally determined limits on in/out distance (distance along motor shaft direction)
ANGLE_MIN = -45
ANGLE_MAX = 45
POS_STEP = 0.01
RPY_STEP = .1


class Vector3:
    def __init__(self, val):
        self.a = val

    def __add__(self,b):
        return Vector3([self.a[0]+b[0],self.a[1]+b[1],self.a[2]+b[2]])

    def __sub__(self,b):
        return Vector3([self.a[0]-b[0],self.a[1]-b[1],self.a[2]-b[2]])

    def __mul__(self,b):
        try:
            return Vector3([self.a[0]*b[0],self.a[1]*b[1],self.a[2]*b[2]])
        except TypeError: #assume we got a scalar instead of a vector
            return Vector3([self.a[0]*b, self.a[1]*b, self.a[2]*b])

    def __div__(self,b):
        return Vector3([self.a[0]/b, self.a[1]/b, self.a[2]/b])

    def dot(self,b):
        return self.a[0]*b[0]+self.a[1]*b[1]+self.a[2]*b[2]

    def cross(self, b):
        nx = self.a[1]*b[2]-self.a[2]*b[1]
        ny = self.a[2]*b[0]-self.a[0]*b[2]
        nz = self.a[0]*b[1]-self.a[1]*b[0]
        return Vector3([nx, ny, nz])

    def normalize(self):
        vector_len = sqrt(pow(self.a[0],2)+pow(self.a[1],2)+pow(self.a[2],2))
        if vector_len == 0:
            inv_len = 0
        else:
            inv_len = 1.0/vector_len
        self.a = [self.a[0]*inv_len, self.a[1]*inv_len, self.a[2]*inv_len]
        return self

    def length(self):
        return sqrt(pow(self.a[0],2)+pow(self.a[1],2)+pow(self.a[2],2))

    def val(self):
        return self.a

    def __getitem__(self, n):
        return self.a[n]
    
    def __setitem__(self, n, v):
        self.a[n] = v

    def __repr__(self):
        return str(map(lambda d: round(d,2), self.a))

    def __neg__(self):
        return Vector3([-self.a[0],-self.a[1], -self.a[2]])

    #takes a list of len 3 for vector & axis, and a floating point angle (in radians)
    #returns a list of len 3 that is rotated vector
    def rotate(self, axis, angle):
        sa = sin(angle)
        ca = cos(angle)
        x = self.a[0]
        y = self.a[1]
        z = self.a[2]
        
        #first normalize axis
        axis2 = Vector3(axis)
        axis2.normalize()
        
        # create rotation matrix
        m = [0]*9
        m[ 0 ] = ca + (1 - ca) * axis2[0] * axis2[0];
        m[ 1 ] = (1 - ca) * axis2[0] * axis2[1] - sa * axis2[2];
        m[ 2 ] = (1 - ca) * axis2[2] * axis2[0] + sa * axis2[1];
        m[ 3 ] = (1 - ca) * axis2[0] * axis2[1] + sa * axis2[2];
        m[ 4 ] = ca + (1 - ca) * axis2[1] * axis2[1];
        m[ 5 ] = (1 - ca) * axis2[1] * axis2[2] - sa * axis2[0];
        m[ 6 ] = (1 - ca) * axis2[2] * axis2[0] - sa * axis2[1];
        m[ 7 ] = (1 - ca) * axis2[1] * axis2[2] + sa * axis2[0];
        m[ 8 ] = ca + (1 - ca) * axis2[2] * axis2[2];

        self.a = [m[0] * x + m[1] * y + m[2] * z,
                  m[3] * x + m[4] * y + m[5] * z,
                  m[6] * x + m[7] * y + m[8] * z]
        return self

class Hexapod:
    def __init__(self):
        self.wrists = [0]*6
        self.shoulders = [0]*6
        self.elbows = [0]*6
        self.shoulder_to_elbow = [0]*6
        self.ee_pos = 0
        self.ee_rpy = 0
        self.ee_up = 0
        self.ee_fw = 0
        self.ee_left = 0
        self.angles = [0]*6
        self.rel_z = 0


        self.update_end_effector(0,0,0,0,0,0)
        self.build_shoulders()
        self.update_wrists()

        el = self.elbows[0]
        wr = self.wrists[0]
        aa = el[1]-wr[1]
        cc = FOREARM_LENGTH
        bb = sqrt(cc*cc-aa*aa)
        self.rel_z = bb+B2S_Z-T2W_Z

        self.update_ik(0,0,0,0,0,0)


    def update_end_effector(self, x,y,z,u,v,w):
        # update end effector position
        self.ee_pos = Vector3([x,y,z])
        self.ee_rpy = Vector3([radians(u), radians(v), radians(w)])
        self.ee_up = Vector3([0,0,1])
        self.ee_fw = Vector3([1,0,0])
        self.ee_lft = Vector3([0,1,0])

            #roll
        axis = Vector3([1,0,0])
        self.ee_up.rotate(axis, self.ee_rpy[0])
        self.ee_fw.rotate(axis, self.ee_rpy[0])
        self.ee_lft.rotate(axis, self.ee_rpy[0])

            #pitch
        axis = Vector3([0,1,0])
        self.ee_up.rotate(axis, self.ee_rpy[1])
        self.ee_fw.rotate(axis, self.ee_rpy[1])
        self.ee_lft.rotate(axis, self.ee_rpy[1])

            #yaw
        axis = Vector3([0,0,1])
        self.ee_up.rotate(axis, self.ee_rpy[2])
        self.ee_fw.rotate(axis, self.ee_rpy[2])
        self.ee_lft.rotate(axis, self.ee_rpy[2])

    def build_shoulders(self):
       #get shoulder and elbow initial positions
        for i in range(0,3):
            c = cos(i*pi*2/3.0)
            s = sin(i*pi*2/3.0)

            n = self.ee_fw
            o = self.ee_up.cross(self.ee_fw)
            o.normalize()

            n1 = n*c + o*s
            o1 = n*(-s) + o*c

            self.shoulders[i*2+0] = n1*B2S_X - o1*B2S_Y + self.ee_up*B2S_Z
            self.shoulders[i*2+1] = n1*B2S_X + o1*B2S_Y + self.ee_up*B2S_Z

            self.elbows[i*2+0] = n1*B2S_X - o1*(B2S_Y+BICEP_LENGTH) + self.ee_up*B2S_Z
            self.elbows[i*2+1] = n1*B2S_X + o1*(B2S_Y+BICEP_LENGTH) + self.ee_up*B2S_Z

            self.shoulder_to_elbow[i*2+0] = -o1
            self.shoulder_to_elbow[i*2+1] = o1

    def update_wrists(self):
        #update wrist positions
        for i in range(0,3):
            c = cos(i*pi*2/3.0)
            s = sin(i*pi*2/3.0)

            n = self.ee_fw
            o = self.ee_up.cross(self.ee_fw)
            o.normalize()

            n1 = n*c + o*s
            o1 = n*(-s) + o*c

            self.wrists[i*2+0] = self.ee_pos + n1*T2W_X - o1*T2W_Y + self.ee_up*T2W_Z
            self.wrists[i*2+1] = self.ee_pos + n1*T2W_X + o1*T2W_Y + self.ee_up*T2W_Z


    def update_ik(self, x, y, z, u, v, w):
        #print "Updating IK for pose: " + str(x) + " " + str(y) + " " + str(z) + " " + str(u) + " " + str(v) + " " + str(w)
        z = z+self.rel_z
        self.update_end_effector(x,y,z,u,v,w)
        self.update_wrists()
        self.update_shoulders()



    def update_shoulders(self):
        for i in range(0,6):
            ortho = Vector3([cos((i/2)*2*pi/3.0),sin((i/2)*2*pi/3.0), 0])
            w = self.wrists[i]-self.shoulders[i]
            a = w.dot(ortho)

            wop = w - (ortho*a)

            b = sqrt(FOREARM_LENGTH*FOREARM_LENGTH-a*a)
            r1 = b
            r0 = BICEP_LENGTH
            d = wop.length()

            a = (r0*r0 - r1*r1 + d*d)/(2*d)

            wop = wop/d

            temp = self.shoulders[i]+(wop*a)

            hh = sqrt(r0*r0-a*a)

            r = ortho.cross(wop)

            if i%2 == 0:
                self.elbows[i]=temp + r*hh
            else:
                self.elbows[i]=temp - r*hh

            temp = self.elbows[i]-self.shoulders[i]
            y = -temp[2]
            temp[2] = 0
            x = temp.length()

            if self.shoulder_to_elbow[i].dot(temp) < 0:
                x = -x

            self.angles[i] = degrees(atan2(-y,x))

    def get_rpy(self):
        return copy.deepcopy(self.ee_rpy)

    def get_pos(self):
        ret = copy.deepcopy(self.ee_pos)
        ret[2] = ret[2]-self.rel_z
        return ret


    def check_ik(self, x=None,y=None,z=None,u=None,v=None,w=None):
        old_pos = self.get_pos()
        old_rpy = self.get_rpy()
        
        if x==None:
            x = old_pos[0]
        if y==None:
            y = old_pos[1]
        if z==None:
            z = old_pos[2]
        if u==None:
            u = old_rpy[0]
        if v==None:
            v = old_rpy[1]
        if w==None:
            w = old_rpy[2]

        #print "Checking IK for pose: " + str(x) + " " + str(y) + " " + str(z) + " " + str(u) + " " + str(v) + " " + str(w)

        success = True
        try:
            self.update_ik(x,y,z,u,v,w)
        except ValueError:
            success = False

        #ik is possible given arm lengths
        #now consider limits on angle

        for i in range(0,6):
            if self.angles[i] < ANGLE_MIN or self.angles[i] > ANGLE_MAX:
                success = False
            
        self.update_ik(old_pos[0],old_pos[1],old_pos[2],old_rpy[0],old_rpy[1],old_rpy[2])
        return success
        

    def nearest_valid_pose(self, x=None,y=None,z=None,u=None,v=None,w=None):
        old_pos = self.get_pos()
        old_rpy = self.get_rpy()
        
        if x==None:
            x = old_pos[0]
        if y==None:
            y = old_pos[1]
        if z==None:
            z = old_pos[2]
        if u==None:
            u = old_rpy[0]
        if v==None:
            v = old_rpy[1]
        if w==None:
            w = old_rpy[2]

        success = self.check_ik(x,y,z,u,v,w)
        changed = False
        while not success:
            goal_pos = Vector3([x,y,z])
            goal_rpy = Vector3([u,v,w])

            zero_pos = Vector3([0,0,0])
            zero_rpy = Vector3([0,0,0])

            step_pos = zero_pos-goal_pos
            step_rpy = zero_pos-goal_rpy

            step_pos.normalize()
            step_pos = step_pos*POS_STEP
            step_rpy.normalize()
            step_rpy = step_rpy*RPY_STEP
            
            new_goal_pos = goal_pos+step_pos
            new_goal_rpy = goal_rpy+step_rpy

            x = new_goal_pos[0]
            y = new_goal_pos[1]
            z = new_goal_pos[2]

            u = new_goal_rpy[0]
            v = new_goal_rpy[1]
            w = new_goal_rpy[2]

            success = self.check_ik(x,y,z,u,v,w)
            changed = True

        return (x,y,z,u,v,w)

    def best_effort_ik(self, x=None,y=None,z=None,u=None,v=None,w=None):
        old_pos = self.get_pos()
        old_rpy = self.get_rpy()
        
        (x1,y1,z1,u1,v1,w1) = self.nearest_valid_pose(x,y,z,u,v,w)

        self.update_ik(x1,y1,z1,u1,v1,w1)
        return self.angles

if __name__ == '__main__':
    v = [0,0,1]
    a = [0,1,0]
    ang = pi/2
    #print rotate_vector3(v, a, ang)
    h = Hexapod()

    

    

    
