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
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

"""
Driver for reading data from the PyGame API. Used from Inpyt.py for reading input data.
"""

__author__ = 'Bitcraze AB'
__all__ = ['PyGameReader']

import time

"""
Linux joystick driver using the Linux input_joystick subsystem. Requires sysfs
to be mounted on /sys and /dev/input/js* to be readable.

This module is very linux specific but should work on any CPU platform
"""

import struct
import glob
import os
import platform
import ctypes
import fcntl

#Constants
TYPE_BUTTON = 1
TYPE_AXIS = 2

class JEvent(object):
    """
    Joystick event class. Encapsulate single joystick event.
    """
    def __init__(self, type, number, value):
        self.type = type
        self.number = number
        self.value = value

    def __repr__(self):
        return "JEvent(type={}, number={}, value={})".format(self.type,
                   self.number, self.value)

#from .constants import TYPE_BUTTON, TYPE_AXIS
#from .jevent import JEvent

if platform.system() != 'Linux':
    raise ImportError("This driver works on Linux only")

JS_EVENT_FMT = "@IhBB"
JE_TIME = 0
JE_VALUE = 1
JE_TYPE = 2
JE_NUMBER = 3


JS_EVENT_BUTTON = 0x001
JS_EVENT_AXIS = 0x002
JS_EVENT_INIT = 0x080

#ioctls
JSIOCGAXES = 0x80016a11
JSIOCGBUTTONS = 0x80016a12


class Joystick():
    """
    Linux jsdev implementation of the Joystick class
    """

    def __init__(self):
        self.opened = False
        self.buttons = []
        self.axes = []
        self.jsfile = None
        self.device_id = -1
        self.devices = {}
        self.start = 0
        self.end = 0
        
    def devices_num(self):
        return len(self.devices)
    
    def devices_numes(self):
        return self.devices.keys()
    
    def device_name(self, device_id):
        if device_id in self.devices:
            return self.devices[device_id]
        return ""

    def available_devices(self):
        """
        Returns a dict with device_id as key and device name as value of all
        the detected devices.
        """
        self.devices = {}

        syspaths = glob.glob("/sys/class/input/js*")

        for path in syspaths:
            device_id = int(os.path.basename(path)[2:])
            with open(path + "/device/name") as namefile:
                name = namefile.read().strip()
            self.devices[device_id] = name
        
        return self.devices

    def open(self, device_id):
        """
        Open the joystick device. The device_id is given by available_devices
        """
        if self.opened:
            return
            raise Exception("A joystick is already opened")

        self.device_id = device_id

        self.jsfile = open("/dev/input/js{}".format(self.device_id), "r")
        fcntl.fcntl(self.jsfile.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        
        val = ctypes.c_int()
        if fcntl.ioctl(self.jsfile.fileno(), JSIOCGAXES, val) != 0:
            self.jsfile.close()
            raise Exception("Failed to read number of axes")
        self.axes = list(0 for i in range(val.value))

        if fcntl.ioctl(self.jsfile.fileno(), JSIOCGBUTTONS, val) != 0:
            self.jsfile.close()
            raise Exception("Failed to read number of axes")
        self.buttons = list(0 for i in range(val.value))

        self.__initvalues()

        self.opened = True
        print self.devices[device_id]

    def close(self):
        """Open the joystick device"""
        if not self.opened:
            return

        self.jsfile.close()
        self.opened = False

    def __initvalues(self):
        """Read the buttons and axes initial values from the js device"""
        for _ in range(len(self.axes) + len(self.buttons)):
            data = self.jsfile.read(struct.calcsize(JS_EVENT_FMT))
            jsdata = struct.unpack(JS_EVENT_FMT, data)
            self.__updatestate(jsdata)

    def __updatestate(self, jsdata):
        """Update the internal absolute state of buttons and axes"""
        if jsdata[JE_TYPE] & JS_EVENT_AXIS != 0:
            if jsdata[JE_NUMBER] in {0, 1, 2, 3, 16, 17, 18, 19}:#all the axes
                self.axes[jsdata[JE_NUMBER]] = jsdata[JE_VALUE] / 32768.0
        if jsdata[JE_TYPE] & JS_EVENT_BUTTON != 0:
            self.buttons[jsdata[JE_NUMBER]] = jsdata[JE_VALUE]
        #define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
        #define PS3_AXIS_STICK_LEFT_UPWARDS      1
        #define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
        #define PS3_AXIS_STICK_RIGHT_UPWARDS     3
        #define PS3_AXIS_BUTTON_CROSS_UP         4
        #define PS3_AXIS_BUTTON_CROSS_RIGHT      5
        #define PS3_AXIS_BUTTON_CROSS_DOWN       6
        #define PS3_AXIS_BUTTON_CROSS_LEFT       7
        #define PS3_AXIS_BUTTON_REAR_LEFT_2      8
        #define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
        #define PS3_AXIS_BUTTON_REAR_LEFT_1      10
        #define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
        #define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
        #define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
        #define PS3_AXIS_BUTTON_ACTION_CROSS     14
        #define PS3_AXIS_BUTTON_ACTION_SQUARE    15
        #define PS3_AXIS_ACCELEROMETER_LEFT      16
        #define PS3_AXIS_ACCELEROMETER_FORWARD   17
        #define PS3_AXIS_ACCELEROMETER_UP        18
        #define PS3_AXIS_GYRO_YAW                19
    
    def __decode_event(self, jsdata):
        """ Decode a jsdev event into a dict """
        #TODO: Add timestamp?
        if jsdata[JE_TYPE] & JS_EVENT_AXIS != 0:
            if jsdata[JE_NUMBER] in {0, 1, 2, 3, 16, 17, 18, 19}:#all the axes
                return JEvent(type=TYPE_AXIS, number=jsdata[JE_NUMBER], value=jsdata[JE_VALUE] / 32768.0)
        if jsdata[JE_TYPE] & JS_EVENT_BUTTON != 0:
            return JEvent(type=TYPE_BUTTON, number=jsdata[JE_NUMBER], value=jsdata[JE_VALUE] / 32768.0)
        return None

    def get_events(self):
        """ Returns a list of all joystick event since the last call """
        if not self.opened:
            raise Exception("Joystick device not opened")

        events = []

        while True:
            try:
                data = self.jsfile.read(struct.calcsize(JS_EVENT_FMT))
            except IOError:  # Raised when there are nothing to read
                break
            jsdata = struct.unpack(JS_EVENT_FMT, data)
            self.__updatestate(jsdata)
            self.end = time.clock()
            event = self.__decode_event(jsdata)
            if event is not None:
                events.append(event)

        return events
    
    def get_axis(self):
        """ Returns a list of all joystick event since the last call """
        return self.axes

class PyGameReader():
    
    """Used for reading data from input devices using the PyGame API."""
    def __init__(self):
        self.inputMap = None
        self.linuxjsdev = Joystick()

    def start_input(self, deviceId, inputMap):
        """Initalize the reading and open the device with deviceId and set the mapping for axis/buttons using the
        inputMap"""
        self.data = {"roll":0.0, "pitch":0.0, "yaw":0.0, "thrust":0.0, "joy_yaw":0.0, "pitchcal":0.0, "rollcal":0.0, "estop": False, "exit":False, "althold":False, "flipleft":False, "flipright":False, "calibrate":False, "switchmode":False,}
        self.inputMap = inputMap
        self.linuxjsdev.available_devices()
        self.linuxjsdev.open(deviceId)

    def read_input(self):
        """Read input from the selected device."""
        # We only want the pitch/roll cal to be "oneshot", don't
        # save this value.
        self.data["pitchcal"] = 0.0
        self.data["rollcal"]  = 0.0
        
        for e in self.linuxjsdev.get_events():
          if e.type == TYPE_AXIS:
            index = "Input.AXIS-%d" % e.number 
            try:
                if (self.inputMap[index]["type"] == "Input.AXIS"):
                    key = self.inputMap[index]["key"]
                    axisvalue = e.value
                    # All axis are in the range [-a,+a]
                    axisvalue = axisvalue * self.inputMap[index]["scale"]
                    # The value is now in the correct direction and in the range [-1,1]
                    self.data[key] = axisvalue
            except Exception:
                # Axis not mapped, ignore..
                pass        
        
          if e.type == TYPE_BUTTON:
            index = "Input.BUTTON-%d" % e.number            
            try:
                if (self.inputMap[index]["type"] == "Input.BUTTON"):
                    key = self.inputMap[index]["key"]
                    if (key == "estop"):
                        self.data["estop"] = not self.data["estop"]
                    elif (key == "exit"):
                        self.data["exit"] = True
                    elif (key == "althold"):
                        self.data["althold"] = not self.data["althold"]   
                    elif (key == "flipleft"):
                        self.data["flipleft"] = not self.data["flipleft"]   
                    elif (key == "flipright"):
                        self.data["flipright"] = not self.data["flipright"]    
                    elif (key == "calibrate"):
                        self.data["calibrate"] = not self.data["calibrate"]   
                    elif (key == "switchmode"):
                        self.data["switchmode"] = not self.data["switchmode"]                        
                    else: # Generic cal for pitch/roll
                        self.data[key] = self.inputMap[index]["scale"]
            except Exception:
                # Button not mapped, ignore..
                pass
          '''
          # if i use the joypad i MUST disable althold...
          if e.type == TYPE_BUTTON and e.value == 0:
            index = "Input.BUTTON-%d" % e.number            
            try:
                if (self.inputMap[index]["type"] == "Input.BUTTON"):
                    key = self.inputMap[index]["key"]
                    if (key == "althold"):
                        self.data["althold"] = False     
                    if (key == "flipleft"):
                        self.data["flipleft"] = False     
                    if (key == "flipright"):
                        self.data["flipright"] = False   
                    if (key == "calibrate"):
                        self.data["calibrate"] = False     
                    if (key == "switchmode"):
                        self.data["switchmode"] = False                     
            except Exception:
                # Button not mapped, ignore..
                pass   
            '''         
        return self.data

    def enableRawReading(self, deviceId):
        """Enable reading of raw values (without mapping)"""
        self.linuxjsdev.open(deviceId)
        #self.j.init()

    def disableRawReading(self):
        """Disable raw reading"""
        # No need to de-init since there's no good support for multiple input devices
        self.linuxjsdev.close()
        pass

    def readRawValues(self):
        """Read out the raw values from the device"""
        rawaxis = {}
        rawbutton = {}

        for e in self.linuxjsdev.get_events():
            if e.type == TYPE_BUTTON:
                rawbutton[e.number] = e.value
            if e.type == TYPE_AXIS:
                rawaxis[e.number] = e.value

        return [rawaxis,rawbutton]

    def getAvailableDevices(self):
        """List all the available devices."""
        dev = []
        self.linuxjsdev.available_devices()
        names = []
        nbrOfInputs = self.linuxjsdev.devices_num()
        ids = self.linuxjsdev.devices_numes()
        for i in range(0,nbrOfInputs):
            id = ids[i]
            name = self.linuxjsdev.device_name(id)
            if names.count(name) > 0:
                name = "{0} #{1}".format(name, names.count(name) + 1)
            dev.append({"id":id, "name" : name})
            names.append(name)
        return dev

