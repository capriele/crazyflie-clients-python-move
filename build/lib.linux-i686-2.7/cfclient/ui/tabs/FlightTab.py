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
The flight control tab shows telimitry data and flight settings.
"""

__author__ = 'Bitcraze AB'
__all__ = ['FlightTab']

import sys

import logging
logger = logging.getLogger(__name__)

from time import time

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import Qt, pyqtSlot, pyqtSignal, QThread, SIGNAL
from PyQt4.QtGui import QMessageBox

from cflib.crazyflie import Crazyflie

from cfclient.ui.widgets.ai import AttitudeIndicator
from cfclient.ui.widgets.compass import CompassWidget

from cfclient.utils.guiconfig import GuiConfig
from cflib.crazyflie.log import Log, LogVariable, LogConfig

from cfclient.ui.tab import Tab
from cfclient.utils.freefall import FreeFallDetection as FFD          
from cfclient.utils.freefall import FreeFallRecovery as FFR 

import math
from math import pi, sin, cos

flight_tab_class = uic.loadUiType(sys.path[0] + "/cfclient/ui/tabs/flightTab.ui")[0]

MAX_THRUST = 65365.0


class FlightTab(Tab, flight_tab_class):

    uiSetupReadySignal = pyqtSignal()
    
    _mode_index = 0

    _motor_data_signal = pyqtSignal(int, object, object)
    _acc_data_signal = pyqtSignal(int, object, object)
    _imu_data_signal = pyqtSignal(int, object, object)
    _althold_data_signal = pyqtSignal(int, object, object)
    _baro_data_signal = pyqtSignal(int, object, object)
    _mag_data_signal = pyqtSignal(int, object, object)

    _input_updated_signal = pyqtSignal(float, float, float, float)
    _rp_trim_updated_signal = pyqtSignal(float, float)
    _emergency_stop_updated_signal = pyqtSignal(bool)
    _switch_mode_updated_signal = pyqtSignal()

    _log_error_signal = pyqtSignal(object, str)
    
    # UI_DATA_UPDATE_FPS = 10

    connectionFinishedSignal = pyqtSignal(str)
    disconnectedSignal = pyqtSignal(str)

    def __init__(self, tabWidget, helper, *args):
        super(FlightTab, self).__init__(*args)
        self.setupUi(self)
        self.count = 0
        
        self.setStyleSheet("QToolTip { color: #ffffff; background-color: #2d2d2d; border: 1px solid #f0f0f0; border-radius: 3px; }")
        
        self.apitch = 0
        self.aroll = 0
        self.motor_power = 0
        self.tabName = "Flight Control"
        self.menuName = "Flight Control"
        
        self.ai = AttitudeIndicator()
        self.compass = CompassWidget()

        self.tabWidget = tabWidget
        self.helper = helper
        
        ########## Freefall related stuff          
        self.ffd = FFD(parent=self) # detection class          
        self.ffr = FFR(parent=self) # recovery class          
               
        # Connect the crash/free fall detections          
        self.ffd.sigFreeFall.connect(self.freefallDetected)          
        self.ffd.sigCrashed.connect(self.crashDetected)          
        self.ffr.sigRecoveryTimedOut.connect(self.recoveryAborted)          
               
        # continuously send acc, mean, var to AI          
        self.ffd.sigAccMeanVar.connect(self.ai.setFFAccMeanVar)          
        # Only set recovery on if both fall and crash detection are on as well as recovery          
        self.checkBox_ffd.stateChanged.connect(lambda on: self.ffGuiSync(0,on) )  # detect freefall on/off          
        self.checkBox_crash.stateChanged.connect(lambda on: self.ffGuiSync(1,on)) # detect crashing on/off          
        self.checkBox_ffr.stateChanged.connect(lambda on: self.ffGuiSync(2,on))   # recovery crashing on/off          
               
        # Use barometer for recovery (only clickedable if the checkbox activates in reaction to detecting a 10DOF flie          
        self.checkBox_ffbaro.clicked.connect(self.ffr.setUseBaro)          
                          
        # intercept control commands          
        self.helper.inputDeviceReader.auto_input_updated.add_callback(self.ffr.step)          
        self.ffr.auto_input_updated.add_callback(self.helper.cf.commander.send_setpoint)          
        self.ffr.auto_input_updated.add_callback(self._input_updated_signal.emit)          
        self.ffr.althold_updated.add_callback(lambda param, arg: self.helper.cf.param.set_value(param, str(arg)))          
               
        # Emergency Stop          
        self._emergency_stop_updated_signal.connect(self.ffr.setKillSwitch)          
        #self._emergency_stop_updated_signal.connect(self.ai.setKillSwitch)          
               
        # Debugging Freefall          
        self.pushButton_ff.clicked.connect(self.ffd.sendFakeEmit)          
        self.pushButton_crash.clicked.connect(self.ffd.sendFakeLandingEmit)          
        self.doubleSpinBox_ff_falloff.valueChanged.connect(self.ffr.falloff.setWidth)          
        self.doubleSpinBox_ff_max.valueChanged.connect(self.ffr.falloff.setMaxThrust)          
        self.doubleSpinBox_ff_min.valueChanged.connect(self.ffr.falloff.setMinThrust)          
        self.doubleSpinBox_ff_time.valueChanged.connect(self.ffr.falloff.setTime)          
        self.pushButton_plot.clicked.connect(self.ffr.falloff.plot)          
               
        self.checkBox_debug.stateChanged.connect(self.toggleFFDebug)          
        self.checkBox_ffbaro.clicked.connect(self.toggleFFDebug)          
               
        # Slow down drawing to GUi items by keeping track of last received data time          
        self.lastImuTime = 0 

        self.disconnectedSignal.connect(self.disconnected)
        self.connectionFinishedSignal.connect(self.connected)
        # Incomming signals
        self.helper.cf.connected.add_callback(
            self.connectionFinishedSignal.emit)
        self.helper.cf.disconnected.add_callback(self.disconnectedSignal.emit)

        self._input_updated_signal.connect(self.updateInputControl)
        self.helper.inputDeviceReader.input_updated.add_callback(
                                     self._input_updated_signal.emit)
        self._rp_trim_updated_signal.connect(self.calUpdateFromInput)
        self.helper.inputDeviceReader.rp_trim_updated.add_callback(
                                     self._rp_trim_updated_signal.emit)
        self._emergency_stop_updated_signal.connect(self.updateEmergencyStop)
        self.helper.inputDeviceReader.emergency_stop_updated.add_callback(self._emergency_stop_updated_signal.emit)
        
        self._switch_mode_updated_signal.connect(self.switchMode)
        self.helper.inputDeviceReader.switch_mode_updated.add_callback(self._switch_mode_updated_signal.emit)
        
        self.helper.inputDeviceReader.althold_updated.add_callback(self.changeHoldmode)
        
        #YAW POINTER
        self.helper.inputDeviceReader.pointer_input_updated.add_callback(self.pointerYawChanged)   

        self._imu_data_signal.connect(self._imu_data_received)
        self._baro_data_signal.connect(self._baro_data_received)
        self._althold_data_signal.connect(self._althold_data_received)
        self._motor_data_signal.connect(self._motor_data_received)
        self._acc_data_signal.connect(self._acc_data_received)
        self._mag_data_signal.connect(self._mag_data_received)

        self._log_error_signal.connect(self._logging_error)

        # Connect UI signals that are in this tab
        self.flightModeCombo.currentIndexChanged.connect(self.flightmodeChange)
        self.minThrust.valueChanged.connect(self.minMaxThrustChanged)
        self.maxThrust.valueChanged.connect(self.minMaxThrustChanged)
        self.maxAltitude.valueChanged.connect(self.maxAltitudeChanged)
        self.thrustLoweringSlewRateLimit.valueChanged.connect(
                                      self.thrustLoweringSlewRateLimitChanged)
        self.slewEnableLimit.valueChanged.connect(
                                      self.thrustLoweringSlewRateLimitChanged)
        self.targetCalRoll.valueChanged.connect(self._trim_roll_changed)
        self.targetCalPitch.valueChanged.connect(self._trim_pitch_changed)
        self.maxAngle.valueChanged.connect(self.maxAngleChanged)
        self.maxYawRate.valueChanged.connect(self.maxYawRateChanged)
        self.uiSetupReadySignal.connect(self.uiSetupReady)
        self.clientHoldModeCheckbox.toggled.connect(self.changeHoldmode)
        self.isInCrazyFlightmode = False
        self.uiSetupReady()
        
        self.viscousModeSlider.valueChanged.connect(self.setViscousThrust)
        self.clientHoldModeCheckbox.setChecked(GuiConfig().get("client_side_holdmode"))

        # self.crazyflieXModeCheckbox.clicked.connect(
        #                     lambda enabled:
        #                     self.helper.cf.param.set_value("flightmode.x", str(enabled)))
        # self.helper.cf.param.add_update_callback(
        #                group="flightmode", name="xmode",
        #                cb=( lambda name, checked:
        #                self.crazyflieXModeCheckbox.setChecked(eval(checked))))
        self.ratePidRadioButton.clicked.connect(
                    lambda enabled:
                    self.helper.cf.param.set_value("flightmode.ratepid",
                                                   str(enabled)))
        self.angularPidRadioButton.clicked.connect(
                    lambda enabled:
                    self.helper.cf.param.set_value("flightmode.ratepid",
                                                   str(not enabled)))
        self.helper.cf.param.add_update_callback(
                    group="flightmode", name="ratepid",
                    cb=(lambda name, checked:
                    self.ratePidRadioButton.setChecked(eval(checked))))
        
        self.helper.cf.param.add_update_callback(
                    group="flightmode", name="althold",
                    cb=(lambda name, enabled:
                    self.helper.inputDeviceReader.setAltHold(eval(enabled))))

        self.helper.cf.param.add_update_callback(
                        group="imu_sensors",
                        cb=self._set_available_sensors)
                
        self.logBaro = None
        self.logAltHold = None

        self.horizontalLayout_5.addWidget(self.ai)
        self.horizontalLayout_5.addWidget(self.compass)
        # self.verticalLayout_4.addWidget(self.ai)
        self.splitter.setSizes([1000, 1])
        

        self.targetCalPitch.setValue(GuiConfig().get("trim_pitch"))
        self.targetCalRoll.setValue(GuiConfig().get("trim_roll"))
        
    def setViscousThrust(self, thrust):
        self.helper.inputDeviceReader.setViscousModeThrust(thrust)
        point = QtGui.QCursor.pos()
        QtGui.QToolTip.showText(QtCore.QPoint(point.x(), self.viscousModeSlider.pos().y()*2-12), QtCore.QString.number(thrust) + "%")
    
    @pyqtSlot(int)          
    def toggleFFDebug(self, ignored=None):          
            if self.checkBox_debug.isChecked():          
                self.pushButton_ff.show()          
                self.pushButton_crash.show()          
               
                if self.checkBox_ffbaro.isEnabled() and self.checkBox_ffbaro.isChecked():          
                    self.pushButton_plot.hide()          
                    self.doubleSpinBox_ff_falloff.hide()          
                    self.doubleSpinBox_ff_max.hide()          
                    self.doubleSpinBox_ff_min.hide()          
                    self.doubleSpinBox_ff_time.hide()          
                    self.label_ff4.hide()          
                    self.label_ff1.hide()          
                    self.label_ff2.hide()          
                    self.label_ff3.hide()          
                else:          
                    self.pushButton_plot.show()          
                    self.doubleSpinBox_ff_falloff.show()          
                    self.doubleSpinBox_ff_max.show()          
                    self.doubleSpinBox_ff_min.show()          
                    self.doubleSpinBox_ff_time.show()          
                    self.label_ff4.show()          
                    self.label_ff1.show()          
                    self.label_ff2.show()          
                    self.label_ff3.show()          
            else:          
                self.pushButton_ff.hide()          
                self.pushButton_crash.hide()          
                self.pushButton_plot.hide()          
                self.doubleSpinBox_ff_falloff.hide()          
                self.doubleSpinBox_ff_max.hide()          
                self.doubleSpinBox_ff_min.hide()          
                self.doubleSpinBox_ff_time.hide()          
                self.label_ff4.hide()          
                self.label_ff1.hide()          
                self.label_ff2.hide()          
                self.label_ff3.hide()          
               
               
    @pyqtSlot()          
    def freefallDetected(self):          
            self.ai.setFreefall()          
            if self.checkBox_ffr.isChecked() and self.checkBox_ffr.isEnabled():          
                self.ffr.startRecovery()          
                self.ai.setRecovery(True)          
                self.helper.inputDeviceReader.setAuto(True)          
                self.checkBox_ffr.setChecked(False)          
                # self.emergency_stop_label.setText("Recovering Freefall")          
            # self.checkBox_ffr.setEnabled(False)          
               
    @pyqtSlot(float)          
    def crashDetected(self, badness):          
            self.ai.setRecovery(False, 'Landed / Crashed')          
            self.ai.setCrash(badness)          
            self.helper.inputDeviceReader.setAuto(False)          
            self.ffr.setLanded()          
            # self.emergency_stop_label.setText("")          
            self.checkBox_ffr.setChecked(True)          
               
    @pyqtSlot()          
    def recoveryAborted(self):          
            self.ai.setRecovery(False, 'Recovery Aborted / Timed out')          
            self.helper.inputDeviceReader.setAuto(False)          
            # self.emergency_stop_label.setText("")          
            # self.checkBox_ffr.setEnabled(True)          
            pass          
               
               
    def ffGuiSync(self, id, on):          
            """ Keeps freefall gui elements in sync"""          
            if   id == 0:  # freefall detection click          
                self.ffd.setEnabled(on)  # detect freefall on/off          
            elif id == 1:  # landing detection click          
                self.ffd.setEnabledLanding(on)  # detect crashing on/off          
            # elif id == 2: # recovery detection click          
               
               
            # Only allow recivery checkbox to be turned on if crash/freefall detection is active          
            self.checkBox_ffr.setEnabled(self.checkBox_ffd.isChecked() and self.checkBox_crash.isChecked())          
               
            # Only allow recovery class to be turned on if all boxes are checked          
            # self.ffr.setEnabled(on=self.checkBox_ffd.isChecked() and self.checkBox_crash.isChecked() and self.checkBox_ffr.isChecked() and self.checkBox_ffr.isEnabled())          
               
               
    def makeSpace(self):          
            # I wish I knew how to make the ai QWidget fullscreen....          
            s = QtGui.QSplitter()          
            if self.splitter_2.widget(0).isHidden():          
                self.splitter_2.widget(0).show()          
                self.splitter.widget(1).show()          
            else:          
                self.splitter_2.widget(0).hide()          
                self.splitter.widget(1).hide()          
               
    def _acc_data_received(self, timestamp, data, logconf): 
            self.ffd.readAcc(data["acc.zw"])    
            self.helper.cf.commander.setActualGravity(data)
                  
               
               
    def updateCamList(self):          
            """Repopulate the combobox with available camera devices and select the last one (=usually the camera we need)"""          
               
            # Remove all          
            self.comboBox_camera.clear()          
               
            # Query available devices          
            devices = self.cam.getDevices()          
            for i, device in enumerate(devices):          
                logger.debug("Camera device [%d]: %s", i, device)          
                self.comboBox_camera.addItem(device)          
               
            # enable/disable the on/off button and dropdown          
            self.checkBox_camera.setEnabled(len(devices) > 0)          
            self.comboBox_camera.setEnabled(len(devices) > 0)          
            if len(devices) == 0:          
                self.comboBox_camera.addItem("None Detected")          
               
            # Select last one (eg the one we just plugged in)          
            self.comboBox_camera.setCurrentIndex(max(0, len(devices) - 1)) 

    def thrustToPercentage(self, thrust):
        return ((thrust / MAX_THRUST) * 100.0)

    def uiSetupReady(self):
        flightComboIndex = self.flightModeCombo.findText(
                             GuiConfig().get("flightmode"), Qt.MatchFixedString)
        if (flightComboIndex < 0):
            self.flightModeCombo.setCurrentIndex(0)
            self.flightModeCombo.currentIndexChanged.emit(0)
        else:
            self.flightModeCombo.setCurrentIndex(flightComboIndex)
            self.flightModeCombo.currentIndexChanged.emit(flightComboIndex)

    def _logging_error(self, log_conf, msg):
        QMessageBox.about(self, "Log error", "Error when starting log config"
                " [%s]: %s" % (log_conf.name, msg))

    def _motor_data_received(self, timestamp, data, logconf):
        self.motor_power = data["motor.m1"] + data["motor.m2"] + data["motor.m3"] + data["motor.m4"]
        if self.isVisible():
            self.actualM1.setValue(data["motor.m1"])
            self.actualM2.setValue(data["motor.m2"])
            self.actualM3.setValue(data["motor.m3"])
            self.actualM4.setValue(data["motor.m4"])
        
    def _baro_data_received(self, timestamp, data, logconf):
        self.helper.inputDeviceReader.setCurrentAltitude(data["baro.aslLong"])
        if self.isVisible():
            self.actualASL.setText(("%.2f" % data["baro.aslLong"]))
            self.ai.setBaro(data["baro.aslLong"])
        
    def _althold_data_received(self, timestamp, data, logconf):
        if self.isVisible():
            target = data["altHold.target"]
            if target > 0:
                if not self.targetASL.isEnabled():
                    self.targetASL.setEnabled(True) 
                self.targetASL.setText(("%.2f" % target))
                self.ai.setHover(target)    
            elif self.targetASL.isEnabled():
                self.targetASL.setEnabled(False)
                self.targetASL.setText("Not set")   
                self.ai.setHover(0)    
        
    def _imu_data_received(self, timestamp, data, logconf):
        self.aroll = data["stabilizer.roll"]  
        self.apitch = data["stabilizer.pitch"]
        if self.isVisible():
            self.actualRoll.setText(("%.2f" % data["stabilizer.roll"]))
            self.actualPitch.setText(("%.2f" % data["stabilizer.pitch"]))
            self.actualYaw.setText(("%.2f" % data["stabilizer.yaw"]))
            self.actualThrust.setText("%.2f%%" % 
                                      self.thrustToPercentage(
                                                      data["stabilizer.thrust"]))
    
            self.ai.setRollPitch(-data["stabilizer.roll"], data["stabilizer.pitch"])
            self.compass.setAngle(-data["stabilizer.yaw"])
            #self.helper.cf.commander.setActualPoint(data)
            
    def _mag_data_received(self, timestamp, data, logconf):              
        # hard and soft correction values generated by Processing Magnetometer_calibration script + calibrate_hardsoft.py
        magn_ellipsoid_center = [1341.66, -537.690, 41.1584]
        magn_ellipsoid_transform = [[0.934687, 0.0222809, 0.0151145], [0.0222809, 0.919365, -0.00622367], [0.0151145, -0.00622367, 0.996487]]                                

        # values generated by calibrate_powered.py
        qx = [0.067946222436498283, -0.25034004667098259, 8.3336994198409666, -0.17762637163222378]
        qy = [-0.13945102271766135, 2.9074808469097495, 1.6764850422889934, 0.19244505046927501]
        qz = [0.018800599305554239, -0.79590273035713055, -3.1033531112103478, 0.13550993988096199]
                
        # matrix by vector multiplication
        def mv(a, b):
            out = [0,0,0]
            for x in range(0, 3):
                out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
            return out 
        
        # calculate adjustments related to how much power is sent to the motors      
        def adj(qs, power):
            p = float(power) / float(40000) # 10k * 4 motors
            return qs[0]*p**3+qs[1]*p**2+qs[2]*p+qs[3]

        x, y, z = data['mag.x'], data['mag.y'], data['mag.z']

        x = x - magn_ellipsoid_center[0]
        y = y - magn_ellipsoid_center[1]
        z = z - magn_ellipsoid_center[2]
        x, y, z = mv(magn_ellipsoid_transform, [x, y, z])

        x = x + adj(qx, self.motor_power)
        y = y + adj(qy, self.motor_power)
        z = z + adj(qz, self.motor_power)
        
        # correct magnetometer orientation relative to the CF orientation
        x, y, z = y, x, z * -1

        # calculate tilt-compensated heading angle        
        cosRoll = cos(math.radians(self.aroll))
        sinRoll = sin(math.radians(self.aroll))  
        cosPitch = cos(math.radians(self.apitch))
        sinPitch = sin(math.radians(self.apitch))
  
        Xh = x * cosPitch + z * sinPitch
        Yh = x * sinRoll * sinPitch + y * cosRoll - z * sinRoll * cosPitch
  
        heading = math.atan2(Yh, Xh)
        d_heading = math.degrees(heading) * -1 # for some reason getting inveted sign here
        
        # update compass widget
        #self.compass.setAngle(d_heading)

        # lock heading to 0 degrees north
        if d_heading > 0:
            yaw_trim = -20
        else:
            yaw_trim = 20
        self.helper.inputDeviceReader.update_trim_yaw_signal(yaw_trim)

    def connected(self, linkURI):
        # IMU & THRUST
        lg = LogConfig("Stabalizer", GuiConfig().get("ui_update_period"))
        lg.add_variable("stabilizer.roll", "float")
        lg.add_variable("stabilizer.pitch", "float")
        lg.add_variable("stabilizer.yaw", "float")
        lg.add_variable("stabilizer.thrust", "uint16_t")
        #lg.add_variable("acc.x", "float")
        #lg.add_variable("acc.y", "float")
        #lg.add_variable("acc.z", "float")

        self.helper.cf.log.add_config(lg)
        if (lg.valid):
            lg.data_received_cb.add_callback(self._imu_data_signal.emit)
            lg.error_cb.add_callback(self._log_error_signal.emit)
            lg.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")
            
        # MOTOR
        lg = LogConfig("Motors", GuiConfig().get("ui_update_period"))
        lg.add_variable("motor.m1")
        lg.add_variable("motor.m2")
        lg.add_variable("motor.m3")
        lg.add_variable("motor.m4")

        self.helper.cf.log.add_config(lg)
        if lg.valid:
            lg.data_received_cb.add_callback(self._motor_data_signal.emit)
            lg.error_cb.add_callback(self._log_error_signal.emit)
            lg.start()
        else:
            logger.warning("Could not setup logconfiguration after "
                           "connection!")
        
        # Acc for freefall
        lg = LogConfig("Acc", 10) # Yes we need this at 100hz for freefall detection!!
        lg.add_variable("acc.zw", "float")

        self.helper.cf.log.add_config(lg)
        if lg.valid:
            lg.data_received_cb.add_callback(self._acc_data_signal.emit)
            lg.error_cb.add_callback(self._log_error_signal.emit)
            lg.start()
        else:
            logger.warning("Could not setup logconfiguration after connection [ACC]!")
        
        lg = LogConfig("Magnetometer", 100)
        lg.add_variable("mag.x", "int16_t")
        lg.add_variable("mag.y", "int16_t")
        lg.add_variable("mag.z", "int16_t")
            
        self.helper.cf.log.add_config(lg)
        if lg.valid:
            lg.data_received_cb.add_callback(self._mag_data_signal.emit)
            lg.error_cb.add_callback(self._log_error_signal.emit)
            lg.start()
        else:
            logger.warning("Could not setup logconfiguration after connection!")
            
    def _set_available_sensors(self, name, available):
        logger.info("[%s]: %s", name, available)
        available = eval(available)
        if ("HMC5883L" in name):
            if (not available):
                self.maxAltitudeLabel.setEnabled(False)
                self.maxAltitude.setEnabled(False)
                self.actualASL.setText("N/A")
                self.actualASL.setEnabled(False)
                self.checkBox_ffbaro.setEnabled(False)       
                self.checkBox_ffbaro.setChecked(False) 
            else:
                self.actualASL.setEnabled(True)
                self.checkBox_ffbaro.setEnabled(True)
                self.helper.inputDeviceReader.setAltHoldAvailable(available)
                if (not self.logBaro and not self.logAltHold):
                    # The sensor is available, set up the logging
                    self.logBaro = LogConfig("Baro", 200)
                    self.logBaro.add_variable("baro.aslLong", "float")

                    self.helper.cf.log.add_config(self.logBaro)
                    if self.logBaro.valid:
                        self.logBaro.data_received_cb.add_callback(self._baro_data_signal.emit)
                        self.logBaro.error_cb.add_callback(self._log_error_signal.emit)
                        self.logBaro.start()
                    else:
                        logger.warning("Could not setup logconfiguration after "
                                       "connection!")            
                    self.logAltHold = LogConfig("AltHold", 200)
                    self.logAltHold.add_variable("altHold.target", "float")

                    self.helper.cf.log.add_config(self.logAltHold)
                    if self.logAltHold.valid:
                        self.logAltHold.data_received_cb.add_callback(self._althold_data_signal.emit)
                        self.logAltHold.error_cb.add_callback(self._log_error_signal.emit)
                        self.logAltHold.start()
                    else:
                        logger.warning("Could not setup logconfiguration after "
                                       "connection!")                        

    def disconnected(self, linkURI):
        self.ai.setRollPitch(0, 0)
        self.actualM1.setValue(0)
        self.actualM2.setValue(0)
        self.actualM3.setValue(0)
        self.actualM4.setValue(0)
        self.actualRoll.setText("")
        self.actualPitch.setText("")
        self.actualYaw.setText("")
        self.actualThrust.setText("")
        self.actualASL.setText("")
        self.targetASL.setText("Not Set")
        self.targetASL.setEnabled(False)
        self.actualASL.setEnabled(False)
        self.logBaro = None
        self.logAltHold = None

    def minMaxThrustChanged(self):
        self.helper.inputDeviceReader.set_thrust_limits(self.minThrust.value(), self.maxThrust.value())
        if (self.isInCrazyFlightmode == True):
            GuiConfig().set("min_thrust", self.minThrust.value())
            GuiConfig().set("max_thrust", self.maxThrust.value())
            
    def maxAltitudeChanged(self):
        self.helper.inputDeviceReader.setMaxAltitude(self.maxAltitude.value())

    def thrustLoweringSlewRateLimitChanged(self):
        self.helper.inputDeviceReader.set_thrust_slew_limiting(
                            self.thrustLoweringSlewRateLimit.value(),
                            self.slewEnableLimit.value())
        if (self.isInCrazyFlightmode == True):
            GuiConfig().set("slew_limit", self.slewEnableLimit.value())
            GuiConfig().set("slew_rate", self.thrustLoweringSlewRateLimit.value())

    def maxYawRateChanged(self):
        logger.debug("MaxYawrate changed to %d", self.maxYawRate.value())
        self.helper.inputDeviceReader.set_yaw_limit(self.maxYawRate.value())
        if (self.isInCrazyFlightmode == True):
            GuiConfig().set("max_yaw", self.maxYawRate.value())

    def maxAngleChanged(self):
        logger.debug("MaxAngle changed to %d", self.maxAngle.value())
        self.helper.inputDeviceReader.set_rp_limit(self.maxAngle.value())
        if (self.isInCrazyFlightmode == True):
            GuiConfig().set("max_rp", self.maxAngle.value())

    def _trim_pitch_changed(self, value):
        logger.debug("Pitch trim updated to [%f]" % value)
        self.helper.inputDeviceReader.set_trim_pitch(value)
        GuiConfig().set("trim_pitch", value)

    def _trim_roll_changed(self, value):
        logger.debug("Roll trim updated to [%f]" % value)
        self.helper.inputDeviceReader.set_trim_roll(value)
        GuiConfig().set("trim_roll", value)

    def calUpdateFromInput(self, rollCal, pitchCal):
        logger.debug("Trim changed on joystick: roll=%.2f, pitch=%.2f",
                     rollCal, pitchCal)
        self.targetCalRoll.setValue(rollCal)
        self.targetCalPitch.setValue(pitchCal)

    def updateInputControl(self, roll, pitch, yaw, thrust):
        self.targetRoll.setText(("%0.2f" % roll))
        self.targetPitch.setText(("%0.2f" % pitch))
        self.targetYaw.setText(("%0.2f" % yaw))
        self.targetThrust.setText(("%0.2f %%" % 
                                   self.thrustToPercentage(thrust)))
        self.thrustProgress.setValue(thrust)

    def setMotorLabelsEnabled(self, enabled):
        self.M1label.setEnabled(enabled)
        self.M2label.setEnabled(enabled)
        self.M3label.setEnabled(enabled)
        self.M4label.setEnabled(enabled)

    def emergencyStopStringWithText(self, text):
        return ("<html><head/><body><p>"
                "<span style='font-weight:600; color:#7b0005;'>{}</span>"
                "</p></body></html>".format(text))

    def updateEmergencyStop(self, emergencyStop):
        if emergencyStop:
            self.setMotorLabelsEnabled(False)
            self.emergency_stop_label.setEnabled(True)
            self.emergency_stop_label.setText(self.emergencyStopStringWithText("Kill Switch Active"))
            self.ai.setKillSwitch(True)
        else:
            self.setMotorLabelsEnabled(True)
            self.emergency_stop_label.setText("Kill Swith")
            self.emergency_stop_label.setEnabled(False)
            self.ai.setKillSwitch(False)

    def flightmodeChange(self, item):
        GuiConfig().set("flightmode", self.flightModeCombo.itemText(item))
        logger.info("Changed flightmode to %s",
                    self.flightModeCombo.itemText(item))
        self.isInCrazyFlightmode = False
        if (item == 0):  # Normal
            self.maxAngle.setValue(GuiConfig().get("normal_max_rp"))
            self.maxThrust.setValue(GuiConfig().get("normal_max_thrust"))
            self.minThrust.setValue(GuiConfig().get("normal_min_thrust"))
            self.slewEnableLimit.setValue(GuiConfig().get("normal_slew_limit"))
            self.thrustLoweringSlewRateLimit.setValue(
                                              GuiConfig().get("normal_slew_rate"))
            self.maxYawRate.setValue(GuiConfig().get("normal_max_yaw"))
        if (item == 1):  # Advanced
            self.maxAngle.setValue(GuiConfig().get("max_rp"))
            self.maxThrust.setValue(GuiConfig().get("max_thrust"))
            self.minThrust.setValue(GuiConfig().get("min_thrust"))
            self.slewEnableLimit.setValue(GuiConfig().get("slew_limit"))
            self.thrustLoweringSlewRateLimit.setValue(
                                                  GuiConfig().get("slew_rate"))
            self.maxYawRate.setValue(GuiConfig().get("max_yaw"))
            self.isInCrazyFlightmode = True

        if (item == 0):
            newState = False
        else:
            newState = True
        self.maxThrust.setEnabled(newState)
        self.maxAngle.setEnabled(newState)
        self.minThrust.setEnabled(newState)
        self.thrustLoweringSlewRateLimit.setEnabled(newState)
        self.slewEnableLimit.setEnabled(newState)
        self.maxYawRate.setEnabled(newState)
        
    def pointerYawChanged(self, yawValue, forced):
        self.compass.setAngle2(yawValue)
        if self.count % 2 == 0 or forced:
            #print "SET POINTER YAW: " + str(-yawValue)
            self.helper.cf.commander.setPointerYaw(yawValue)
            #self.helper.cf.param.set_value("FlightMode.referenceNorth", str(-yawValue))
        self.count += 1
        
    @pyqtSlot(bool)
    def changeCareFreemode(self, checked):
        # self.helper.cf.commander.set_client_carefreemode(checked)
        if checked:
            self.clientCareFreeModeRadio.setChecked(checked)
            self.helper.cf.param.set_value("FlightMode.flightmode", "0")
            # self.helper.cf.commander.set_client_positionmode(False)
            # self.helper.cf.commander.set_client_xmode(False)
        elif not checked and self._mode_index == 1:
            self.clientCareFreeModeRadio.setChecked(False)
            self.clientNormalModeRadio.setChecked(True)
            self.helper.cf.param.set_value("FlightMode.flightmode", "1")
        logger.info("CareFree-mode enabled: %s", checked)

    @pyqtSlot(bool)
    def changeXmode(self, checked):
        self.clientXModeRadio.setChecked(checked)
        # self.helper.cf.commander.set_client_xmode(checked)
        if checked:
            self.helper.cf.param.set_value("FlightMode.flightmode", "2")
            # self.helper.cf.commander.set_client_carefreemode(False)
            # self.helper.cf.commander.set_client_positionmode(False)
        logger.info("X-mode enabled: %s", checked)
        
    @pyqtSlot(bool)
    def changePositionmode(self, checked):
        self.clientPositionModeRadio.setChecked(checked)
        # self.helper.cf.commander.set_client_positionmode(checked)
        if checked:
            self.helper.cf.param.set_value("FlightMode.flightmode", "3")
            # self.helper.cf.commander.set_client_carefreemode(False)
            # self.helper.cf.commander.set_client_xmode(False)
        logger.info("Position-mode enabled: %s", checked)
        
    @pyqtSlot(bool)
    def changeHeadingmode(self, checked):
        self.clientHeadingModeRadio.setChecked(checked)
        # self.helper.cf.commander.set_client_positionmode(checked)
        if checked:
            self.helper.cf.param.set_value("FlightMode.flightmode", "4")
            # self.helper.cf.commander.set_client_carefreemode(False)
            # self.helper.cf.commander.set_client_xmode(False)
        logger.info("Heading-mode enabled: %s", checked)
        
    @pyqtSlot(bool)
    def changeHoldmode(self, checked):
        self.clientHoldModeCheckbox.setChecked(checked)
        self.helper.cf.param.set_value("flightmode.althold", str(checked))
        # self.helper.cf.commander.set_client_holdmode(checked)
        GuiConfig().set("client_side_holdmode", checked)
        logger.info("Hold-mode enabled: %s", checked)
    
    def switchMode(self):
        if self._mode_index == 4:
            self._mode_index = 0 
        else: 
            self._mode_index += 1
        if self._mode_index == 0:
            self.changeCareFreemode(True)
            self.changeXmode(False)
            self.changePositionmode(False)
            self.changeHeadingmode(False)
        elif self._mode_index == 1:
            self.changeCareFreemode(False)
            self.changeXmode(False)
            self.changePositionmode(False)
            self.changeHeadingmode(False)
        elif self._mode_index == 2:
            self.changeCareFreemode(False)
            self.changeXmode(True)
            self.changePositionmode(False)
            self.changeHeadingmode(False)
        elif self._mode_index == 3:
            self.changeCareFreemode(False)
            self.changeXmode(False)
            self.changePositionmode(True)
            self.changeHeadingmode(False)
        elif self._mode_index == 4:
            self.changeCareFreemode(False)
            self.changeXmode(False)
            self.changePositionmode(False)
            self.changeHeadingmode(True)
        logger.info("Mode switched to index: %d", self._mode_index)
