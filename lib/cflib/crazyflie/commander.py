#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Used for sending control setpoints to the Crazyflie
"""

__author__ = 'Bitcraze AB'
__all__ = ['Commander']

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
import struct
import math
#from cfclient.utils.input import JoystickReader #used for hold-mode

class Commander():
    """
    Used for sending control setpoints to the Crazyflie
    """

    def __init__(self, crazyflie=None):
        """
        Initialize the commander object. By default the commander is in
        +-mode (not x-mode).
        """
        self._cf = crazyflie

        #########################
        ##        MODES        ##
        #########################
        #self._x_mode = False
        #self._carefree_mode = True
        #self._position_mode = False
        #self._hold_mode = False

        #########################
        ##        UTILS        ##
        #########################
        self.pointerYaw = None
        self._actualGravity = None
        #self._yaw = 0            #used to convert copter's current yaw to radians
        #self._actualPoint = None #copter's current position update every 100ms
        #self._oldThrust = 0
        #self._delta = 5          #used to stabilize the copter
    '''
    def set_client_xmode(self, enabled):
        """
        Enable/disable the client side X-mode. When enabled this recalculates
        the setpoints before sending them to the Crazyflie.
        """
        self._x_mode = enabled

    #def set_client_carefreemode(self, enabled):
        """
        Enable/disable the client side CareFree-mode. 
        When enabled this recalculates the setpoints before sending them to the Crazyflie
        so that the copter direction is indipendent from its current yaw.
        """
        self._carefree_mode = enabled

    def set_client_positionmode(self, enabled):
        """
        Enable/disable the client side Position-mode. 
        When enabled this recalculates the setpoints before sending them to the Crazyflie
        so that the copter manteins current position.
        N.W.: the user can only control the throttle!!!
        """
        self._position_mode = enabled

    def set_client_holdmode(self, enabled):
        """
        Enable/disable the client side Position-mode. 
        When enabled this recalculates the setpoints before sending them to the Crazyflie
        so that the copter manteins current position.
        N.W.: the user can control the copter normally but when he release the button the copter 
              will mantain its current position 
        """
        self._hold_mode = enabled

    def setActualPoint(self, data):
        self._actualPoint = data
    '''
    def setActualGravity(self, data):
        self._actualGravity = data
    def setPointerYaw(self, data):
        self.pointerYaw = data
        
    def send_setpoint(self, roll, pitch, yaw, thrust):
        """
        Send a new control setpoint for roll/pitch/yaw/thust to the copter

        The arguments roll/pitch/yaw/trust is the new setpoints that should
        be sent to the copter
        """
            
        if self.pointerYaw is not None:#elif = else if
            # roll = x, pitch = y, yaw = A, A >= 0
            # x' = x*cos(A) - y*sin(A)
            # y' = x*sin(A) + y*cos(A)
            # A < 0
            # x' =  x*cos(A) + y*sin(A)
            # y' = -x*sin(A) + y*cos(A)
            currentYaw = self.pointerYaw
            self._yaw = math.radians(currentYaw)
            cosy = math.cos(self._yaw)
            siny = math.sin(self._yaw)

            #print "Roll: %3.3f -- Pitch: %3.3f -- Yaw: %3.3f" % (self._actualPoint["stabilizer.roll"], self._actualPoint["stabilizer.pitch"], currentYaw)
            #print "Degree Yaw: %3.3f -- Radians Yaw: %3.3f" % (currentYaw, self._yaw)

            roll1 = roll
            #if self._yaw >= 0:
            #    roll  = roll*cosy - pitch*siny
            #    pitch = roll1*siny + pitch*cosy
            #else:
            roll  = roll*cosy + pitch*siny
            pitch = pitch*cosy - roll1*siny

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        pk.data = struct.pack('<fffH', roll, -pitch, yaw, thrust)
        self._cf.send_packet(pk)
        
    @staticmethod
    def deadband(value, threshold):
        if abs(value) < threshold:
            value = 0
        elif value > 0:
            value -= threshold
        elif value < 0:
            value += threshold
        return value/(1-threshold)
        