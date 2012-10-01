import sys
from PySide import QtGui, QtCore
from PySide.QtGui import QPalette
import numpy as np
class CalibrationGrid(QtGui.QWidget):
	_width  = 700
	_height = 500
	padding = 100
	origin  = None
	corners = []
	
	key_handlers = dict()
	
	def __init__(self, nRows=5, nCols=5, origin=None, scale=1):
		super(CalibrationGrid, self).__init__()
		self.nRows = nRows
		self.nCols = nCols
		self.origin = origin
		self.scale  = scale
		self.initUI()
		
	@property
	def width(self):
		return self.size().width()

	@property
	def height(self):
		return self.size().height()
		
	def escHandler(self, e):
		sys.exit(0)
		
	def initUI(self):
		self.addKeyHandler(16777216, self.escHandler)	   
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
				
	def keyPressEvent(self, e):
		for key, fn in self.key_handlers.items():
			if key == e.key():
				fn(e)
		
	def addKeyHandler(self, key, fn):
		self.key_handlers[key] = fn
		
	def drawRectangles(self, qp):
		black = False
		cref = np.array((255,255,255))
		square = None
		if self.height < self.width:
			square = self.height/self.nRows - 2*self.padding/self.nRows
		else:
			square = self.width/self.nCols - 2*self.padding/self.nCols
			
		square *= self.scale
			
		grid_size_rows = square * self.nRows + 2 * self.padding
		grid_size_cols = square * self.nCols + 2 * self.padding
		
		row_offset = (self.height - grid_size_rows) / 2
		col_offset = (self.width  - grid_size_cols) / 2
		
		top0, left0 = (self.padding + row_offset, self.padding + col_offset)
		if self.origin is not None:
			top0, left0 = self.origin[1], self.origin[0]
        
		del self.corners[:]
		for row in range(self.nRows):
			top = row*square + top0
			for col in range(self.nCols):			
				left = col*square + left0
				if (col < self.nCols - 1) and (row < self.nRows - 1):
					self.corners.append((top+square, left+square))
				color = QtGui.QColor(*(cref*black))
				qp.setPen(color)
				qp.setBrush(color)
				qp.drawRect(left, top, square, square)
			
				black = (not black) 
		
	def getPatternAsImage(self, im_type='PIL'):
		pixmap = QtGui.QPixmap.grabWidget(self)
		qimage = pixmap.toImage()
		if im_type == 'PIL':
			import Image
			pil_im = Image.frombuffer('RGBA', (self.width, self.height), qimage.bits(), 'raw', 'RGBA', 0, 1).convert('L')
			return pil_im
		elif im_type == 'OPENCV':
			import cv
			cv_im = cv.CreateImageHeader((self.width, self.height), cv.IPL_DEPTH_8U, 4)
			cv.SetData(cv_im, qimage.bits())
			return cv_im
