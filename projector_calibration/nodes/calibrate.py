#!/usr/bin/env python
import roslib; roslib.load_manifest('projector_calibration')
from PySide.QtGui import QMainWindow, QPushButton, QApplication, QPainter
import sys

class MainWindow(QMainWindow):
	def __init__(self, parent=None):
		super(MainWindow, self).__init__(parent)

	def paint(self):
		p = QPainter()
		p.begin(self)
		p.drawPoint(100,100)
		p.end()

if __name__ == '__main__':
	app = QApplication(sys.argv)
	frame = MainWindow()
	frame.resize(500,500)
	frame.paint()
	frame.show()
	app.exec_()