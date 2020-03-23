#!/usr/bin/python

#------------------------------------------------------------------------------
# {one line to give the program's name and a brief idea of what it does.}
# Copyright (C) 2017 Kara Douville, Hunter McNichols, and Elaine Schaertl Short
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
from scipy.interpolate import splprep, splev
import serial, math, struct, datetime

class Interpolation:

    def __init__(self):
        pass
            

    def interpolate(self, poses, times):
        this_k = len(poses)-1
        if this_k > 2:
            this_k = 2

        keyframes = [[],[],[],[],[],[]]
        for pose in poses:
            for index in range(6):
                keyframes[index].append(pose[index])
        keytimes = map(lambda t: float(t), times)

        #returnes the tck, and time
        spline = splprep(x = keyframes, u = keytimes, k = this_k, s=.01)

        return spline

    #helper function to evaluate the spline at an array of times
    def eval_spline(self, times, spline):
        ret = splev(times, spline)
        return map(lambda x: map(lambda y: float(y), x), ret)

    def get_spline(self,behavior):
        spline, t = self.interpolated_dict[behavior]
        return spline

    def get_time(self, behavior):
        spline, t = self.interpolated_dict[behavior]
        return t

    #splines to an array of arrays of poses
    def get_poses(self,spline, delta_t):
        time = spline[1]
        times = [time[0] + delta_t * i for i in range(0, int(time[-1]/ delta_t))]
        times.append(time[-1])
        poses = []
        points = self.eval_spline(times, spline[0])
        for i in range(0, len(points[0])):
            p = [points[n][i] for n in range(6)]
            poses.append(p)
        return poses

    
        

    
