#!/usr/bin/python

#------------------------------------------------------------------------------
# Interface to Pololu Micro Maestro 6-channel servo controller with software limits for the SPRITE robot
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


import serial, sys
import rospy

class MotorController:
    def __init__(self, zeros, port): 
        self.test_mode=False
        try:
            self.ser = serial.Serial(port)
            #self.ser.open()
            self.ser.write(chr(0xAA))
            self.ser.flush()
        except serial.serialutil.SerialException as e:
            rospy.logerr("Serial port cannot be opened: "+str(e))
            rospy.logerr("Starting motor controller in testing mode.")
            self.test_mode = True
        self.zeros = zeros

    def set_motor_angle(self, channel, angle):
        ticks = int((angle / 0.09) + self.zeros[channel])
        return self.set_motor_ticks(channel, ticks)

    def set_motor_ticks(self, channel, ticks):
        #make sure target is within physical limits
        if ticks > 2200 or ticks < 800:
            rospy.logwarn("Invalid motor position: " + str(ticks))
            return False
        ticks = ticks * 4   
        serial_bytes = chr(0x84)+chr(channel)+chr(ticks & 0x7F)+chr((ticks >> 7) & 0x7F)
        if not self.test_mode:
            self.ser.write(serial_bytes)
        return True

    def get_motor_angle(self):
        angles = []
        for channel in range(6):
            port.write(chr(0x90) + chr(channel))
            w1 = port.read()
            w2 = port.read()
            target = struct.unpack("<h", w1 + w2)
            target = (((float(list(target)[0]) / 4)- self.zeros[channel]) * 0.09)
            angles.append(target)
        return angles

    def get_motor_ticks(self):
        if self.test_mode:
            rospy.logwarn("Cannot get ticks; in testing mode only.")
            return [0,0,0,0,0,0]
        ticks = []
        for channel in range(6):
            port.write(chr(0x90) + chr(channel))
            w1 = port.read()
            w2 = port.read()
            target = struct.unpack("<h", w1 + w2)
            target = float(list(target)[0]) / 4
            ticks.append(target)
        return ticks

    #speed in deg/s
    def set_speed(self,channel, dps):
        #print dps
        tickspers_goal=(dps / 0.09)
        #print tickspers_goal
        speed = int(0.04*tickspers_goal)
        #print speed

        serial_bytes = chr(0x87)+chr(channel)+chr(speed & 0x7F)+chr((speed >> 7) & 0x7F)
        
        if not self.test_mode:
            self.ser.write(serial_bytes)
        #else:
        #    rospy.logwarn("Setting motor " + str(channel) +" to speed " + str(speed))

    #acc in deg/s^2
    def set_accel(self,channel,dps2):
        tickspers2_goal = (dps2/0.09)
        acc = int(0.0032*tickspers2_goal)

        serial_bytes = chr(0x89)+chr(channel)+chr(acc & 0x7F)+chr((acc >> 7) & 0x7F)
        if not self.test_mode:
            self.ser.write(serial_bytes)
        #else:
        #    rospy.logwarn("Setting motor " + str(channel) +" to acceleration " + str(acc))

    def set_v_all(self,speed):
        for channel in range(6):
            self.set_speed(channel,speed)

    def set_a_all(self,acc):
        for channel in range(6):
            self.set_accel(channel,acc)
   
    def __del__(self):
        if not self.test_mode:
            self.ser.close()

