import numpy as np
from scipy.sparse import lil_matrix
from bisect import bisect_left

class PyCloud(lil_matrix):
	
	def __init__(self, arg1, row_step, point_step, shape=None, dtype=None, copy=False):
		lil_matrix.__init__(self, arg1, shape=shape, dtype=self.PyPoint, copy=copy)
		self.nan = self.PyPoint((np.nan, np.nan, np.nan))
		self.format = 'lil'
		self.dtype = self.PyPoint
		
		
	def __getitem__(self, index):
		item = super(PyCloud, self).__getitem__(index)
		assert type(item) == self.PyPoint or item == 0, 'Expecting %s, got: %s: %s' % (self.dtype.name, str(type(item)), item)
		
		return item.vals if type(item) == self.PyPoint else self.nan
		
	def __setitem__(self, index, item):
		# i, j = index
		if np.ma.extras.issequence(item):
			super(PyCloud, self).__setitem__(index, self.PyPoint(item))
			item = self.PyPoint(item)
		else:
			super(PyCloud, self).__setitem__(index, item)
		
	class PyPoint(np.float64):
		name = "PyPoint"
		
		def __new__(cls, vals):
			return np.float64.__new__(cls, np.nan)
			
		def __init__(self, vals):
			np.float64.__init__(self)
			self.vals = vals
		
		@classmethod
		def type(cls, x):
			if np.ma.extras.issequence(x): 
				return cls(x)
			elif(type(x) == cls):
				return x
			raise TypeError('Unable to convert value (%s) to dtype [%s]' % (x,cls.name))
			
if __name__ == '__main__':
	l = PyCloud((100,100), 8, 8)
	l[5,5] = (1,2,3)
	assert (1,2,3) == l[5,5], 'Wrong value came out'
	assert np.isnan(l[5,7]), 'Unassigned index not nan'