#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_calibration')
from projector_calibration import CalibrationGrid
import PySide
import PySide.QtCore
import sys
import rospy

class GridTest:

    def update(self):
        self.grid.origin = self.grid.origin[0] + 1, self.grid.origin[1] + 1
        self.grid.update()

    def __init__(self):
        app = PySide.QtGui.QApplication(sys.argv)
        self.grid = CalibrationGrid(nCols=7, nRows=5, origin=(50,0), scale= 0.5)
        # grid.addKeyHandler(67, calibrate)
        self.grid.show()
        qtimer = PySide.QtCore.QTimer(self.grid)
        qtimer.timeout.connect(self.update)
        qtimer.start(10)
        sys.exit(app.exec_())
        
if __name__ == '__main__':
    GridTest()