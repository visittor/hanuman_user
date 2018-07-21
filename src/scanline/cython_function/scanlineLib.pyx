from __future__ import division
import numpy as np
cimport numpy as np
cimport cython

DTYPE_INT = np.int 
ctypedef np.int_t DTYPE_INT_t

DTYPE_UINT8 = np.uint8
ctypedef np.uint8_t DTYPE_UINT8_t

DTYPE_FLOAT = np.float 
ctypedef np.float_t DTYPE_FLOAT_t

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef int color_classify_c(int val1, int val2, int val3,np.ndarray[DTYPE_UINT8_t,ndim = 3] color, int n_color):
	cdef int i
	for i in range(0,n_color):
		if color[i, 0, 0]<=val1 and color[i, 1, 0]>=val1:
			if color[i, 0, 1]<=val2 and color[i, 1, 1]>=val2:
				if color[i, 0, 2] <= val3 and color[i, 1, 2] >= val3:
					return i
	return i+1

cdef int approxDistance(int x1, int y1, int x2, int y2):
	cdef int dx, dy
	if x1 - x2 > 0:
		dx =  x1 - x2
	else:
		dx = x2 - x1
	if y1 - y2 > 0:
		dy = y1 - y2
	else:
		dy = y2 - y1
	return dx + dy

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef np.ndarray[DTYPE_INT_t,ndim=2] scan1D_c(np.ndarray[DTYPE_UINT8_t,ndim = 3] img, np.ndarray[DTYPE_UINT8_t,ndim = 3] color_list, int axis, int col, int c, int horizon):
	cdef np.ndarray[DTYPE_INT_t,ndim = 2] temp = np.zeros([img.shape[axis%2],3],dtype = DTYPE_INT)
	cdef int max_ = img.shape[(axis+1)%2]
	cdef int i = max_ - 1
	cdef int i_ = max_ - 1
	cdef int n_color = color_list.shape[0]
	cdef int current_color = -1
	cdef int previous_color = -1
	cdef int x,y
	cdef int counter = 0
	cdef int e = 2
	cdef int step = 1
	while i >= horizon:
		y = i if axis == 1 else col
		x = i if axis == 0 else col
		current_color = color_classify_c(img[y,x,0], img[y,x,1], img[y,x,2],color_list,n_color)
		i_ = i
		if previous_color != current_color:
			previous_color = current_color
			while current_color == previous_color and i_ < max_:
				y = i_ if axis == 1 else col
				x = i_ if axis == 0 else col
				previous_color = color_classify_c(img[y,x,0], img[y,x,1], img[y,x,2],color_list,n_color)
				i_ += 1
		temp[counter,0] = x
		temp[counter,1] = y
		temp[counter,2] = current_color
		counter += 1
		previous_color = current_color
		step = (i - horizon) // c
		step = 1 if step < 1 else step
		i -= step
	out = temp[:counter,:]
	return out

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef np.ndarray[DTYPE_INT_t,ndim=2] scanlinePolygon(np.ndarray[DTYPE_UINT8_t,ndim = 3] img, np.ndarray[DTYPE_UINT8_t,ndim = 3] color_list, np.ndarray[DTYPE_INT_t,ndim=2] polygon,int step):
	cdef int max_h = img.shape[0]
	cdef int max_w = img.shape[1]
	cdef int numPoint = polygon.shape[0]
	cdef int x1,y1,x2,y2,x,y
	cdef int diffX, diffY, maxStep, i, count = 0
	cdef int n_color = color_list.shape[0]
	cdef int current_color = -1
	cdef int index = 0
	cdef np.ndarray[DTYPE_INT_t,ndim=2] temp = np.zeros([max_h*max_w+1,3], dtype=DTYPE_INT)

	while index < numPoint-1:
		x1 = polygon[index,0]
		y1 = polygon[index,1]
		x2 = polygon[index+1,0]
		y2 = polygon[index+1,1]
		diffX = x2 - x1
		diffY = y2 - y1
		maxStep = np.absolute(max(diffX,diffY,key = lambda x:np.absolute(x)))
		i = 0
		while i < maxStep:
			x = x1 + ((diffX*i)//maxStep)
			x = max_w-1 if x>=max_w else x if x > 0 else 0
			y = y1 + ((diffY*i)//maxStep)
			y = max_h-1 if y>=max_h else y if y > 0 else 0
			current_color = color_classify_c(img[y,x,0], img[y,x,1], img[y,x,2],color_list,n_color)
			temp[count,0] = x
			temp[count,1] = y
			temp[count,2] = current_color
			count += 1
			i += step
		index += 1
	out = temp[:count,:]
	return out

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef np.ndarray[DTYPE_INT_t,ndim = 2] scanline2region_c(np.ndarray[DTYPE_INT_t,ndim = 2] scanline, int minPix):
	cdef int max_ = scanline.shape[0]
	cdef int i = 0, counter = 0, pre_i = 0
	cdef int color, pre_color, prepre_i
	cdef np.ndarray[DTYPE_INT_t, ndim = 2] temp = np.zeros((max_+1, 5), dtype = DTYPE_INT)
	temp[counter,0] = scanline[i,0]
	temp[counter,1] = scanline[i,1]
	temp[counter,4] = scanline[i,2]
	color = scanline[i,2]
	pre_color = color
	i += 1
	while i < max_:
		if color != scanline[i,2]:
			if approxDistance(temp[counter,0], temp[counter,1], scanline[i,0], scanline[i,1]) < minPix:
				if counter != 0:
					counter -= 1
					color = pre_color
				else:
					temp[counter,0] = scanline[i,0]
					temp[counter,1] = scanline[i,1]
					temp[counter,4] = scanline[i,2]
					color = scanline[i,2]
					pre_color = color
					pre_i = i
			else:
				temp[counter,2] = scanline[i,0]
				temp[counter,3] = scanline[i,1]
				pre_color = temp[counter,4]
				counter += 1
				temp[counter,0] = scanline[i,0]
				temp[counter,1] = scanline[i,1]
				temp[counter,4] = scanline[i,2]
				color = scanline[i,2]
				pre_i = i
		i += 1
	temp[counter,2] = scanline[i-1,0]
	temp[counter,3] = scanline[i-1,1]
	out = temp[:counter+1, :]
	return out

def scan1D(img, color_list, axis, col, c, horizon = 0):
	return scan1D_c(img, color_list, axis, col, c, horizon)

def scanPolygon(img, color_list, polygon, step = 1):
	return scanlinePolygon(img, color_list, polygon, step)

def scanline2region(scanline, minPix = 1):
	return scanline2region_c(scanline, minPix)

cpdef scan2DVerticle(np.ndarray[DTYPE_UINT8_t,ndim = 3] img, np.ndarray[DTYPE_UINT8_t,ndim = 3] color_list, int grid_dis, int c, int horizon):
	out = []
	cdef int max_col = img.shape[1]
	cdef int col
	for col in range(0, max_col, grid_dis):
		out.append(scan1D_c(img, color_list, 1, col, c, horizon))
	return out

cpdef scan2DHorizon(np.ndarray[DTYPE_UINT8_t,ndim = 3] img, np.ndarray[DTYPE_UINT8_t,ndim = 3] color_list, int grid_dis, int horizon):
	out = []
	cdef int max_col = horizon
	cdef int max_w = img.shape[1]
	cdef int col
	for col in range(0, max_col, grid_dis):
		out.append(scan1D_c(img, color_list, 0, col, np.power(2,max_w), 0))
	return out

def scanlines2regions(scanlines, minPix = 1):
	cdef int i
	cdef int max_ = len(scanlines)
	cdef regions = []
	for i in range(0,max_):
		regions.append(scanline2region_c(scanlines[i], minPix))
	return regions
