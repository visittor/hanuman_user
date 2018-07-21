from __future__ import division
from libc.stdlib cimport rand
import numpy as np
cimport numpy as np
cimport cython

cpdef int ABC = 0
cpdef int MC = 1

cdef extern from "math.h":
	float sqrtf(float m)

###############################################################################
#########function for change a array of point to array of line eqaution########
@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef void fromPoint2mc(np.ndarray[np.int_t, ndim=1] start, np.ndarray[np.int_t, ndim=1] stop, np.ndarray[np.float_t, ndim=1] out):
	cdef float m, c
	if start[0] == stop[0]:
		m = float('inf')
		c = float('inf')
	else:
		m = float(start[1] - stop[1]) / float(start[0] - stop[0])
		c = float(start[1]) - m*float(start[0])
	out[0] = m
	out[1] = c
	out[2] = float(start[0]) if start[0] < stop[0] else float(stop[0])
	out[3] = float(stop[0]) if stop[0] > start[0] else float(start[0])

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef void fromPoint2ABC(np.ndarray[np.int_t, ndim=1] start, np.ndarray[np.int_t, ndim=1] stop, np.ndarray[np.float_t, ndim=1] out):
	cdef float A,B,C
	A = float(start[1] - stop[1])
	B = float(start[0] - stop[0])
	C = (B*float(start[1])) - (A*float(start[0]))
	if A != 0:
		out[0] = A / A
		out[1] = -B / A
		out[2] = C / A
	elif B != 0:
		out[0] = A / B
		out[1] = -B / B
		out[2] = C / B
	else:
		out[0] = A
		out[1] = -B
		out[2] = C
	out[3] = float(start[0]) if start[0] < stop[0] else float(stop[0])
	out[4] = float(stop[0]) if stop[0] > start[0] else float(start[0])


cdef void fromPoints2mc(np.ndarray[np.int_t, ndim=2] starts, np.ndarray[np.int_t, ndim=2] stops, np.ndarray[np.float_t, ndim=2] out):
	cdef int len_point = starts.shape[0]
	cdef int i

	for i in range(len_point):
		fromPoint2mc(starts[i], stops[i], out[i])

cdef void fromPoints2ABC(np.ndarray[np.int_t, ndim=2] starts, np.ndarray[np.int_t, ndim=2] stops, np.ndarray[np.float_t, ndim=2] out):
	cdef int len_point = starts.shape[0]
	cdef int i
	# print starts[0], stops[0]
	for i in range(len_point):
		fromPoint2ABC(starts[i], stops[i], out[i])

cdef void fromPoint2LineEq(np.ndarray[np.int_t, ndim=1] start, np.ndarray[np.int_t, ndim=1] stop, np.ndarray[np.float_t, ndim=1] out, int mode):
	if mode == ABC:
		fromPoint2ABC(start, stop, out)
	elif mode == MC:
		fromPoint2mc(start, stop, out)

cdef void fromPoints2LineEq(np.ndarray[np.int_t, ndim=2] starts, np.ndarray[np.int_t, ndim=2] stops, np.ndarray[np.float_t, ndim=2] out, int mode):
	if mode == ABC:
		fromPoints2ABC(starts, stops, out)
	elif mode == MC:
		fromPoints2mc(starts, stops, out)

cpdef void PyfromPoint2LineEq(np.ndarray[np.int_t, ndim=1] start, np.ndarray[np.int_t, ndim=1] stop, np.ndarray[np.float_t, ndim=1] out, int mode = ABC):
	if mode == ABC:
		if start.shape[0] != 2 or stop.shape[0]:
			raise ValueError("start or stop have more than x and y")
		elif out.shape[0] != 5:
			raise ValueError("output must be an array of length 5")
		fromPoint2ABC(start, stop, out)
	elif mode == MC:
		if start.shape[0] != 2 or stop.shape[0] != 2:
			raise ValueError("start or stop have more than x and y")
		elif out.shape[0] != 4:
			raise ValueError("output must be an array of length 4")
		fromPoint2mc(start, stop, out)

cpdef void PyfromPoints2LineEq(np.ndarray[np.int_t, ndim=2] starts, np.ndarray[np.int_t, ndim=2] stops, np.ndarray[np.float_t, ndim=2] out, int mode = ABC):
	if starts.shape[0] != stops.shape[0] or starts.shape[0] != out.shape[0]:
		raise ValueError("starts.shape[0] != stops.shape[0] or starts.shape[0] != stops.shape[0]")
	elif starts.shape[1] != 2 or stops.shape[1] != 2:
		raise ValueError("start or stop have more than x and y")
	elif mode == ABC:
		if out.shape[1] != 5:
			raise ValueError("output.shape[1] != 5")
		fromPoints2ABC(starts, stops, out)

	elif mode == MC:
		if out.shape[1] != 4:
			raise ValueError("output.shape[1] != 4")
		fromPoints2mc(starts, stops, out)
###############################################################################
###############################################################################

cdef float norm(np.ndarray[np.float_t,ndim=1] a, int dimention):
	cdef int i
	cdef float sum_
	for i in range(dimention):
		sum_ += a[i]*a[i]
	return sqrtf(sum_)

cdef float norm2array(np.ndarray[np.float_t,ndim=1] a, np.ndarray[np.float_t,ndim=1] b, int dimention):
	cdef int i
	cdef float sum_ = 0
	for i in range(dimention):
		sum_ += (a[i]-b[i])*(a[i]-b[i])
	# print sum_
	return sqrtf(sum_)

cdef float findFrechetDistance(np.ndarray[np.float_t,ndim=2] poly1, np.ndarray[np.float_t,ndim=2] poly2, int len_poly):
	cdef float max_d = 0.0, d, min_d = -1.0
	cdef int i,j
	for i in range(len_poly):
		min_d = -1
		for j in range(len_poly):
			d = norm2array(poly1[i], poly2[j], 2)
			if min_d < 0:
				min_d = d
			elif d < min_d:
				min_d = d
		if min_d > max_d:
			max_d = min_d
	# print "frechet", max_d
	return max_d

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef int compaerLinesABC(np.ndarray[np.float_t,ndim=1] L1, np.ndarray[np.float_t,ndim=1] L2, float frechetThr):
	cdef np.ndarray[np.float_t, ndim=2] test1 = np.zeros((2,2), dtype=np.float)
	cdef np.ndarray[np.float_t, ndim=2] test2 = np.zeros((2,2), dtype=np.float)
	if L1[1] == 0 or L2[1] == 0:
		test1[0,1] = 0.0
		test1[0,0] = -L1[2] / L1[0]
		test1[1,1] = 100.0
		test1[1,0] = -L1[2] / L1[0]

		test2[0,1] = 0.0
		test2[0,0] = -L2[2] / L2[0]
		test2[1,1] = 100.0
		test2[1,0] = -L2[2] / L2[0]
	else:
		test1[0,0] = 0.0
		test1[0,1] = -L1[2] / L1[1]
		test1[1,0] = 100.0
		test1[1,1] = (-100.0*L1[0] - L1[2]) / L1[1]

		test2[0,0] = 0.0
		test2[0,1] = -L2[2] / L2[1]
		test2[1,0] = 100.0
		test2[1,1] = (-100.0*L2[0] - L2[2]) / L2[1]
	if findFrechetDistance(test1, test2, 2) < frechetThr:
		return 1
	return 0

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef int compaerLinesMC(np.ndarray[np.float_t,ndim=1] L1, np.ndarray[np.float_t,ndim=1] L2, float frechetThr):
	cdef np.ndarray[np.float_t, ndim=2] test1 = np.zeros((2,2), dtype=np.float)
	cdef np.ndarray[np.float_t, ndim=2] test2 = np.zeros((2,2), dtype=np.float)
	
	test1[0,0] = 0.0
	test1[0,1] = L1[1]
	test1[1,0] = 100.0
	test1[1,1] = 100.0*L1[0] + L1[1]
	test2[0,0] = 0.0
	test2[0,1] = L2[1]
	test2[1,0] = 100.0
	test2[1,1] = 100.0*L2[0] + L2[1]
	# print test1, test2

	if findFrechetDistance(test1, test2, 2) < frechetThr:
		return 1
	return 0

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef void mergeLineABC(np.ndarray[np.float_t,ndim=1] L1, np.ndarray[np.float_t,ndim=1] L2, np.ndarray[np.float_t,ndim=1] out, float w1, float w2):
	out[0] = (w1*L1[0] + w2*L2[0]) / (w1+w2)
	out[1] = (w1*L1[1] + w2*L2[1]) / (w1+w2)
	out[2] = (w1*L1[2] + w2*L2[2]) / (w1+w2)
	out[3] = L1[3] if L1[3] < L2[3] else L2[3]
	out[4] = L1[4] if L1[4] > L2[4] else L2[4]

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef void mergeLineMC(np.ndarray[np.float_t,ndim=1] L1, np.ndarray[np.float_t,ndim=1] L2, np.ndarray[np.float_t,ndim=1] out, float w1, float w2):
	out[0] = (w1*L1[0] + w2*L2[0]) / (w1+w2)
	out[1] = (w1*L1[1] + w2*L2[1]) / (w1+w2)
	out[2] = L1[2] if L1[2] < L2[2] else L2[2]
	out[3] = L1[3] if L1[3] > L2[3] else L2[3]

cdef np.ndarray[np.float_t, ndim=2] groupingLineABC(np.ndarray[np.float_t,ndim=2] lines,float frechetThr):
	cdef int len_line = lines.shape[0]
	cdef np.ndarray[np.float_t, ndim=2] temp = np.zeros((len_line,lines.shape[1]), dtype=np.float)
	cdef np.ndarray[np.int_t, ndim=1] indxs = np.arange(0, len_line, 1, np.int)
	cdef int count=0, i, isMerge = 0, indx1, indx2

	for i in range(len_line-1):
		isMerge = 0
		indx1 = indxs[i]
		if indx1 == -1:
			continue
		for j in range(i+1, len_line):
			indx2 = indxs[j]
			if indx2 == -1:
				continue
			if compaerLinesABC(lines[indx1], lines[indx2], frechetThr):
				mergeLineABC(lines[indx1], lines[indx2], temp[count], float(isMerge+1), 1.0)
				isMerge += 1
				## Delete by overwrite
				indxs[j] = -1
		if isMerge == 0:
			temp[count] = lines[indx1]
		count += 1
	out = temp[:count,:]
	return out

cdef np.ndarray[np.float_t, ndim=2] groupingLineMC(np.ndarray[np.float_t,ndim=2] lines,float frechetThr):
	cdef int len_line = lines.shape[0]
	cdef np.ndarray[np.float_t, ndim=2] temp = np.zeros((len_line,lines.shape[1]), dtype=np.float)
	cdef np.ndarray[np.int_t, ndim=1] indxs = np.arange(0, len_line, 1, np.int)
	cdef int count=0, i, isMerge = 0, indx1, indx2

	for i in range(len_line):
		isMerge = 0
		indx1 = indxs[i]
		if indx1 == -1:
			continue
		for j in range(i+1, len_line):
			indx2 = indxs[j]
			# print i,j
			if indx2 == -1:
				continue
			elif compaerLinesMC(lines[indx1], lines[indx2], frechetThr):
				# print "merge"
				mergeLineMC(lines[indx1], lines[indx2], temp[count], float(isMerge+1), 1.0)
				isMerge += 1
				## Delete by overwrite
				# lines[j,:] = lines[i,:]
				indxs[j] = -1
		if isMerge == 0:
			temp[count] = lines[indx1]
		count += 1
	out = temp[:count,:]
	return out

cpdef np.ndarray[np.float_t, ndim=2] PyGroupingLineABC(np.ndarray[np.float_t,ndim=2] lines,float frechetThr):
	if lines.shape[1] != 5:
		raise ValueError("lines.shape[1] must equal to 5")
	return groupingLineABC(lines, frechetThr)

cpdef np.ndarray[np.float_t, ndim=2] PyGroupingLineMC(np.ndarray[np.float_t,ndim=2] lines,float frechetThr):
	if lines.shape[1] != 4:
		raise ValueError("lines.shape[1] must equal to 4")
	return groupingLineMC(lines, frechetThr)
####################################################################x##########
###############################################################################