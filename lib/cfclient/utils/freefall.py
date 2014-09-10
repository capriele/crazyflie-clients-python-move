_author__ = 'Oliver Dunkley'

from PyQt4.QtCore import pyqtSignal, pyqtSlot
from PyQt4 import QtGui, QtCore
import os
from numpy import var, mean, float32, zeros
import logging
logger = logging.getLogger(__name__)
from cflib.utils.callbacks import Caller
from input import JoystickReader
from math import pi, cos

import matplotlib.pyplot as plt

class FallOff(QtCore.QObject):
    sigEmitXY = pyqtSignal(list,list)
    
    def __init__(self,parent=None):
        super(QtCore.QObject, self).__init__(parent)
        self.time = 1.
        self.minThrust = 20.
        self.maxThrust = 90.
        self.width = 1. # 0.1 to 10

    def f(self, step):
        step = step/self.time*pi/2/100
        return max(self.minThrust/100,(self.maxThrust/100)*cos(step)**self.width)*100

    @pyqtSlot(float)
    def setTime(self, t):
        self.time=t

    @pyqtSlot(float)
    def setMinThrust(self, t):
        self.minThrust=t

    @pyqtSlot(float)
    def setMaxThrust(self, t):
        self.maxThrust=t

    @pyqtSlot(float)
    def setWidth(self, t):
        self.width=t

    @pyqtSlot()
    def plot(self):
        x = range(1,200)
        y = map(self.f, x)
        plt.plot(x,y)
        plt.grid()
        plt.xlabel("Step @ 100HZ")
        plt.ylabel("Throttle %")
        plt.title("Throttle Response Curve")
        plt.axis([0, 200, -1, 101])
        plt.show()

class FreeFallRecovery(QtCore.QObject):
    """ Class to recover from detected freefall"""

    sigRecoveryTimedOut = pyqtSignal()

    def __init__(self,parent=None):
        super(QtCore.QObject, self).__init__(parent)
        self.parent = parent
        self.useBaro = False     # Using barometer for current recovery
        self.useBaroNext = False # Use barometer for next recovery
        self.falling = False     # Free fall Recovery active
        self.kill = False
        self.boostMSec = 45      # Max thrust for first 30ms. State also keeps track of mode, see step()
        self.nr = 0              # step nr during non baro mode
        self.falloff = FallOff(self)

        # why the hell do we mix signals like this?
        self.auto_input_updated = Caller()
        self.althold_updated = Caller()

        self.timerOut = QtCore.QTimer()

    def setUseBaro(self, on):
        """ If we should use the barometer (ie hover mode) to help recover or not for the next fall"""
        msg = " next " if self.falling else " "
        if self.useBaroNext !=on:
            logger.info("Using Barometer for"+msg+"freefall recovery" if on else "Not using Baro for"+msg+"freefall recovery")
        self.useBaroNext = on


    def step(self, rollTrimmed, pitchTrimmed, yaw, thrust):
        """Here we handle our recover; called by the joy driver @ 100hz. We mostly pass everything through, but we modify the throttle"""

        if self.useBaro>0:
            # Barometer Mode
            if self.useBaro==1:
                # Loop Hover mode
                thrust = int(round(-0.5*32767 + 32767))
            elif self.useBaro==2:
                # Set hover mode
                self.useBaro-=1
                self.enableBaro()
                thrust = int(round(-0.5*32767 + 32767))
            else:
                # Boost mode
                self.useBaro-=1
                thrust = JoystickReader.p2t(min(99,70+self.useBaro*1.25))
                logger.info("Thrust: %d", thrust)
        else:
            # Regular mode
            self.nr +=1
            thrust = JoystickReader.p2t(self.falloff.f(self.nr)) #if thrust>0.05 else 0

        self.auto_input_updated.call(rollTrimmed, pitchTrimmed, yaw, thrust)


    def startRecovery(self):
        if self.kill:
            logger.info("Cannot start freefall recovery due to killswitch")
        else:
            logger.info("Starting freefall recovery")
            self.falling = True
            self.useBaro = self.boostMSec = 30 if self.useBaroNext else 0

            # Start the timeout and the handler
            if self.useBaro:
                self.althold_updated.call("altHold.altHoldErrMax", 260.0)
                self.timerOut.singleShot(3000, self.setTimedOut) # 6 second for recovery with barometer
                #self.enableBaro()
            else:
                self.nr = 0
                self.timerOut.singleShot(2000, self.setTimedOut) # 2 second for recovery without barometer

    def setKillSwitch(self, on):
        self.kill = on
        if on:
            self.abort()

    def abort(self):
        """ We have landed, so stop recovering"""
        if self.cleanUp():
            self.sigRecoveryTimedOut.emit()
            logger.info("Aborted Recovery")

    def setLanded(self):
        """ We have landed, so stop recovering"""
        if self.cleanUp():
            logger.info("Recovery stopped due to landing")

    def setTimedOut(self):
        """ Recovering times out before we detected a landing"""
        if self.cleanUp():
            logger.info("Recovery stopped due to timeout")
            self.sigRecoveryTimedOut.emit()

    def cleanUp(self):
        if self.falling:
            self.falling = False
            self.timerOut.stop()
            if self.useBaro:
                self.disableBaro()
            return True
        return False

    def enableBaro(self):
        # Make change in altitude stronger
        self.althold_updated.call("altHold.altHoldErrMax", 260.0)
        self.althold_updated.call("altHold.altHoldChangeSens", 50.0)
        self.althold_updated.call("flightmode.althold", True)

    def disableBaro(self):
        self.althold_updated.call("flightmode.althold", False)
        self.althold_updated.call("altHold.altHoldErrMax", 1.0)
        self.althold_updated.call("altHold.altHoldChangeSens", 200.0)





class FreeFallDetection(QtCore.QObject):
    """ Class to detect freefall from accelerometer data"""
    # Listens to accelerometer data nad emits a freefall signal if detected
    # Max one detection per 1 second (at 100hz)
    # Note that acceleration in this class refers to gravity aligned acceleration measured in G

    # Signal when freefall detected
    sigFreeFall = pyqtSignal()

    # Signal when crash detected
    sigCrashed = pyqtSignal(float) # float corresponds to how bad the crash was. 0 = perfect landing, 1 = crashed, 2 = crazy crash...
    sigAccMeanVar = pyqtSignal(float, float, float)

    def __init__(self,parent=None):
        super(QtCore.QObject, self).__init__(parent)
        ##### FREE FALL DETECTION
        self.on = True
        # Mean and threshold (detect ff)
        self.m = -1#-0.95
        self.mt = 0.025
        self.l = 15 # history length for freefall
        # Variance and threshold (detect ff)
        self.v = 0.0
        self.vt = 0.001

        # Avoid multiple freefall detections in sequence
        self.timeOutFall = False
        self.timeOutFallMSec = 1000#ms
        self.timerFall= QtCore.QTimer()

        ##### CRASH / LAND DETECTION
        self.onL = False
        # Variance threshold (detect land)
        self.vtL = 0.2 # 0.3
        self.lL = 3  # 3 = good for soft surfaces such as a blanket

        # Avoid multiple landed detections in sequence
        self.timeOutLand = False
        self.timeOutLandMSec = 1000#ms
        self.timerLand = QtCore.QTimer()

        ##### COMMON
        # Ring buffer
        self.pastSize = max(self.l, self.lL)
        self.past = zeros(self.pastSize, dtype=float32)
        self.index = 0






    @pyqtSlot(bool)
    def setEnabled(self, on=True):
        if self.on !=on:
            logger.info("FreeFall Detection enabled" if on else "FreeFall Detection Disabled")
        self.on = on

    @pyqtSlot(bool)
    def setEnabledLanding(self, on=True):
        if self.onL !=on:
            logger.info("Landing Detection enabled" if on else "Landing Detection Disabled")
        self.onL = on


    @pyqtSlot()
    def sendFakeEmit(self):
        """Emit a fake signal set to trigger post freefall detection recovery"""
        if self.on:
            acc = self.m+self.mt*2
            m   = self.m+self.mt/2
            v   = self.v+self.vt/2
            logger.info("Emitting fake freefall signal")
            self.detectFreeFall(acc,m,v)

    @pyqtSlot()
    def sendFakeLandingEmit(self):
        """Emit a fake landing signal"""
        if self.onL:
            logger.info("Emitting fake crash signal")
            self.detectLanding(1.1,0.2,1.56)

    @pyqtSlot(float)
    def readAcc(self, acc=0):
        """ Keeps track of acceleration variance and mean"""

        # Update ringbuffer with latest acc reading
        self.past[self.index] = acc
        self.index += 1
        if self.index % self.pastSize == 0:
            self.index = 0

        # Keep Track of mean and variance
        m = mean(self.past) # All entries
        v = var(self.past) # All entries
        lastFive = range(5-self.index, self.index)
        mL = mean(self.past[lastFive])# TODO: dont need
        vL = var(self.past[lastFive]) # Short variance, 5 last entries

        # Do detections
        self.detectFreeFall(acc, m, v)
        self.detectLanding(acc, mL, vL)

        # Emit acc/mean/variance (for the GUI, etc)
        self.sigAccMeanVar.emit(acc, m, v) # need to add v for landing

    def detectFreeFall(self, acc, m, v):
        """Determine if mean and variance correspond to freefall"""
        # We are currently looking out for free falls
        if self.on:
            # We have not just detected a freefall (must be at least X seconds between them)
            if not self.timeOutFall:
                # Mean corresponds to freefall
                if abs(self.m - m) < self.mt:
                    # Variance corresponds to freefall
                    if abs(self.v - v) < self.vt:
                        logger.info("Freefall detected (G: %f, Mean: %f, Variance: %f", acc, m, v)
                        self.sigFreeFall.emit()
                        self.timeOutFall = True
                        self.timerFall.singleShot(self.timeOutFallMSec, self.resetTimeOutFall)
                        #os.system("beep -f 2000 -l 60&")


    def detectLanding(self, acc, m, v):
        # We are currently looking out for landings
        if self.onL:
            # We have not just detected a freefall (must be at least 3 seconds between them)
            if not self.timeOutLand and (not self.on or self.timeOutFall): # Only detect landings after a freefall (unless we are not detecting falls)
                # Mean corresponds to freefall
                landed = v>self.vtL
                if landed:
                    logger.info("Landing detected (G: %f, Mean: %f, Variance: %f", acc, m, v)
                    self.sigCrashed.emit(v+1)
                    self.timeOutLand = True
                    self.timerLand.singleShot(self.timeOutLandMSec, self.resetTimeOutLand)
                    #os.system("beep -f "+str(int(300/self.vtL*v))+" -l 80&")
                    os.system("beep -f "+str(max(200,min(700,int(400*v))))+" -l 80&")

    def setThresh(self, m, mt, v, vt):
        # Mean and threshold
        self.m = m
        self.mt = mt
        # Variance and threshold
        self.v = v
        self.vt = vt

    def resetTimeOutFall(self):
        self.timeOutFall = False

    def resetTimeOutLand(self):
        self.timeOutLand = False
    #def setTimeout(self, t):
    #    self.timeout = t
    #    self.t = min(self.t, t)
    #
    #def setTimeoutLand(self, t):
    #    self.timeoutL = t
    #    self.tL = min(self.tL, t)
