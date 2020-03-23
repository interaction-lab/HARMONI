#!/usr/bin/python

# Elaine Short

import roslib; roslib.load_manifest('cordial_example')
import rospy
from cordial_core import RobotManager

#------------------------------------------------------------------------------
# Example robot use with CoRDial
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



if __name__=="__main__":
    rospy.init_node("CoRDial_example")
    rm = RobotManager("DB1")
    for i in range(1,15):
        rm.say("statement{}".format(i),wait=True)
