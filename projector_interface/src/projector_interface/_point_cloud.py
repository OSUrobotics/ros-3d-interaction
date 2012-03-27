#!/usr/bin/env python

import roslib; roslib.load_manifest('projector_interface')

import ctypes
import math
import struct
import numpy as np
from scipy.sparse import lil_matrix
from pycloud import PyCloud

from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]	   = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

_NP_TYPES = {
	np.dtype('uint8')	:	(PointField.UINT8,	1),
	np.dtype('int8')	:	(PointField.INT8,	1),
	np.dtype('uint16')	:	(PointField.UINT16,	2),
	np.dtype('int16')	:	(PointField.INT16,	2),
	np.dtype('uint32')	:	(PointField.UINT32,	4),
	np.dtype('int32')	:	(PointField.INT32,	4),
	np.dtype('float32')	:	(PointField.FLOAT32,4),
	np.dtype('float64')	:	(PointField.FLOAT64,8)
}

import time

def _float2rgb(x):
	rgb = struct.unpack('I', struct.pack('f', x))[0]
	b = (rgb >> 16) & 0x0000ff;
	g = (rgb >> 8)	& 0x0000ff;
	r = (rgb)		& 0x0000ff;
	return r,g,b
	
def float2rgb(farr):
	assert farr.ndim == 2
	out = np.zeros(farr.shape + (3,))
	for u, col in enumerate(farr):
		for v, val in enumerate(col):
			out[u,v,:] = _float2rgb(val)
	return out

def read_points_np(cloud, field_names=None, skip_nans=False, uvs=[], masked=True):
	reader = read_points(cloud, field_names=field_names, skip_nans=skip_nans, uvs=uvs)
	points = np.array(list(reader))
	reshaped = np.reshape(points, (cloud.height, cloud.width, len(cloud.fields)))
	if masked:
		return np.ma.masked_array(reshaped, np.isnan(reshaped))		
	else:
		return reshaped

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
	assert(cloud)
	fmt = _get_struct_fmt(cloud, field_names)
	width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
	unpack_from = struct.Struct(fmt).unpack_from

	if skip_nans:
		if uvs:
			for u, v in uvs:
				p = unpack_from(data, (row_step * v) + (point_step * u))
				has_nan = False
				for pv in p:
					if isnan(pv):
						has_nan = True
						break
				if not has_nan:
					yield p
		else:
			for v in xrange(height):
				offset = row_step * v
				for u in xrange(width):
					p = unpack_from(data, offset)
					has_nan = False
					for pv in p:
						if isnan(pv):
							has_nan = True
							break
					if not has_nan:
						yield p
					offset += point_step
	else:
		if uvs:
			for u, v in uvs:
				yield unpack_from(data, (row_step * v) + (point_step * u))
		else:
			for v in xrange(height):
				offset = row_step * v
				for u in xrange(width):
					yield unpack_from(data, offset)
					offset += point_step

def create_cloud_xyz32(header, points):
	fields = [PointField('x', 0, PointField.FLOAT32, 1),
			  PointField('y', 4, PointField.FLOAT32, 1),
			  PointField('z', 8, PointField.FLOAT32, 1)]
	return create_cloud(header, fields, points)

def np_to_point_list(cloud, fields):
	if cloud.ndim > 2:
		return cloud.reshape((np.prod(cloud.shape[:2]), len(fields)))
	else:
		return cloud

def create_cloud_np_xyz(points, header):
	dtype, width = _NP_TYPES[points.dtype]
	fields = [PointField('x', width*0, dtype, 1),
			  PointField('y', width*1, dtype, 1),
			  PointField('z', width*2, dtype, 1)]
	return create_cloud_np(header, fields, points)
	

def create_cloud_np(header, fields, points):
	return create_cloud(header, fields, np_to_point_list(points, fields), points.shape)

def create_cloud(header, fields, points, shape=None):
	cloud = PointCloud2()
	cloud.header	   = header
	cloud.height	   = 1
	cloud.width		   = len(points)
	cloud.is_dense	   = False
	cloud.is_bigendian = False
	cloud.fields	   = fields
	fmt				   = _get_struct_fmt(cloud)
	cloud_struct	   = struct.Struct(fmt)
	cloud.point_step   = cloud_struct.size
	cloud.row_step	   = cloud_struct.size * cloud.width

	buffer = ctypes.create_string_buffer(cloud_struct.size * cloud.width)

	point_step, pack_into = cloud.point_step, cloud_struct.pack_into
	offset = 0
	for p in points:
		if not np.ma.is_masked(p):
			pack_into(buffer, offset, *p)
			offset += point_step

	cloud.data = buffer.raw

	# if shape:
	#	cloud.height, cloud.width = shape[:2]
	return cloud

def _create_point_field(cloud, field_names=None):
	nFields = cloud.shape[2]
	if not field_names:
		if nFields == 3:
			field_names = ['x','y','z']
		elif nFields == 4:
			field_names = ['x','y','z','index'] #ugh, this could also be intensity
		elif nFields == 6:
			field_names = ['x','y','z','r','g','b']
		# field_names = ['x','y','z','r','g','b','a'][0:cloud.shape[2]]
	fields = []
	for field_name in field_names:
		field = PointField()

def _get_struct_fmt(cloud, field_names=None):
	fmt = '>' if cloud.is_bigendian else '<'
	offset = 0
	for field in (f for f in sorted(cloud.fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
		if offset < field.offset:
			fmt += 'x' * (field.offset - offset)
			offset = field.offset
		if field.datatype not in _DATATYPES:
			print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
		else:
			datatype_fmt, datatype_length = _DATATYPES[field.datatype]
			fmt	   += field.count * datatype_fmt
			offset += field.count * datatype_length

	return fmt
