'''
Created on Aug 19, 2014

@author: bitcraze
'''
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

class CompassWidget(QWidget):

    angleChanged = pyqtSignal(float)
    
    def __init__(self, parent = None):
    
        QWidget.__init__(self, parent)
        
        self.setStyleSheet("background-color:transparent;");
        self._angle = 0.0
        self._angle2 = 0.0
        self._margins = 10
        self._pointText = {0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S",
                           225: "SW", 270: "W", 315: "NW"}
    
    def paintEvent(self, event):
    
        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        painter.fillRect(event.rect(), self.palette().brush(QPalette.Window))
        self.drawMarkings(painter)
        self.drawNeedle(painter)
        
        painter.end()
    
    def drawMarkings(self, painter):
    
        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)
        
        font = QFont(self.font())
        font.setPixelSize(10)
        metrics = QFontMetricsF(font)
        
        painter.setFont(font)
        
        i = 0
        while i < 360:
            if i == 0:
                painter.setPen(self.palette().color(QPalette.Highlight))
                painter.drawLine(0, -40, 0, -50)
                painter.drawText(-metrics.width(self._pointText[i])/2.0, -52, self._pointText[i])
                painter.setPen(self.palette().color(QPalette.Shadow))
            elif i % 45 == 0:
                painter.drawLine(0, -40, 0, -50)
                painter.drawText(-metrics.width(self._pointText[i])/2.0, -52,
                                 self._pointText[i])
            else:
                painter.drawLine(0, -45, 0, -50)
            
            painter.rotate(15)
            i += 15
        
        painter.restore()
    
    def drawNeedle(self, painter):
    
        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        painter.rotate(self._angle)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)
        
        painter.setPen(QPen(self.palette().color(QPalette.Shadow), 2, Qt.SolidLine))
        painter.setBrush(self.palette().brush(QPalette.Shadow))
        
        r = 8
        c = 25
        c2 = c-r/2
        painter.drawLine(0, -c2, 0, c2)
        painter.drawLine(-c2, 0, c2, 0)
        
        painter.setPen(QPen(Qt.NoPen))
        painter.drawEllipse(QPoint(0,c), r, r)
        painter.drawEllipse(QPoint(-c,0), r, r)
        painter.drawEllipse(QPoint(c,0), r, r)
        
        painter.setBrush(self.palette().brush(QPalette.Highlight))
        
        painter.drawEllipse(QPoint(0,-c), r, r)
        
        painter.restore()
        
        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        painter.rotate(self._angle2)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)
        
        painter.setPen(QPen(self.palette().color(QPalette.Shadow), 0, Qt.SolidLine))
        painter.setBrush(self.palette().brush(QPalette.Highlight))
        painter.drawPolygon(QPolygon([QPoint(-5, -35), QPoint(0, -50), QPoint(5, -35), QPoint(0, -40), QPoint(-5, -35)]))
        
        painter.restore()

    
    def sizeHint(self):
    
        return QSize(150, 150)
    
    def angle(self):
        return self._angle
    
    @pyqtSlot(float)
    def setAngle(self, angle):
        if angle != self._angle:
            self._angle = angle
            self.angleChanged.emit(angle)
            self.update()
    
    @pyqtSlot(float)
    def setAngle2(self, angle2):
        if angle2 != self._angle2:
            self._angle2 = angle2
            self.angleChanged.emit(angle2)
            self.update()
    
    angle = pyqtProperty(float, angle, setAngle)
    
    @pyqtSlot(float)
    def setValue(self, angle):
        self.setAngle(angle)

if __name__ == "__main__":

    app = QApplication(sys.argv)
    
    window = QWidget()
    compass = CompassWidget()
    spinBox = QSpinBox()
    spinBox.setRange(0, 359)
    spinBox.valueChanged.connect(compass.setAngle)
    
    layout = QVBoxLayout()
    layout.addWidget(compass)
    layout.addWidget(spinBox)
    window.setLayout(layout)
    
    window.show()
    sys.exit(app.exec_())