#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette
import numpy as np
class CalibrationGrid(QtGui.QWidget):
	_width = 500
	_height = 500
	nCols = 5
	nRows = 5
	padding = 50
	corners = []
	def __init__(self):
		super(CalibrationGrid, self).__init__()
		
		self.initUI()
		
	@property
	def width(self):
		return self.size().width()

	@property
	def height(self):
		return self.size().height()
		
	def initUI(self):	   
		p = QPalette()
		p.setColor(QPalette.Background, QtGui.QColor(255,255,255))
		self.setPalette(p)
		self.setGeometry(0, 0, self._width, self._height)
		self.setWindowTitle('Projector Calibration')
		self.show()

	def paintEvent(self, e):
		qp = QtGui.QPainter()
		qp.begin(self)
		self.drawRectangles(qp)
		qp.end()
		
	# def resizeEvent(self, e):
		# import pdb; pdb.set_trace()
		
	def keyPressEvent(self, e):
		if e.key() == 16777216:
			sys.exit(0)
		
	def drawRectangles(self, qp):
		black = False
		cref = np.array((255,255,255))
		square = None
		if self.height < self.width:
			square = self.height/self.nRows - 2*self.padding/self.nRows
		else:
			square = self.width/self.nCols - 2*self.padding/self.nCols
			
		del self.corners[:]
		for row in range(self.nRows):
			top = row*square + self.padding
			for col in range(self.nCols):			
				left = col*square + self.padding
				self.corners.append((top+square, left+square))
				color = QtGui.QColor(*(cref*black))
				qp.setPen(color)
				qp.setBrush(color)
				qp.drawRect(left, top, square, square)
			
				black = (not black)	
		
def main():
	app = QtGui.QApplication(sys.argv)
	grid = CalibrationGrid()
	grid.showFullScreen()
	sys.exit(app.exec_())


if __name__ == '__main__':
	main()