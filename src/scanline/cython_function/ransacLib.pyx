from __future__ import division
from libc.stdlib cimport rand
# from libc.math cimport sqrt
import numpy as np
cimport numpy as np
cimport cython

DTYPE_INT = np.int 
ctypedef np.int_t DTYPE_INT_t

DTYPE_UINT8 = np.uint8
ctypedef np.uint8_t DTYPE_UINT8_t

DTYPE_FLOAT = np.float 
ctypedef np.float_t DTYPE_FLOAT_t

cdef extern from "math.h":
	float sqrtf(float m)

ctypedef int (*ransacWeightFunc)(np.ndarray[DTYPE_FLOAT_t,ndim=1],np.ndarray[DTYPE_INT_t,ndim=1])

ctypedef void (*ransacFitFunc)(np.ndarray[DTYPE_INT_t,ndim=2], np.ndarray[DTYPE_FLOAT_t,ndim=1])

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef int cverifyPoint(np.ndarray[DTYPE_FLOAT_t,ndim=1] coeff,np.ndarray[DTYPE_INT_t,ndim=1] point, float T):
	cdef float d, num, den

	num = coeff[0]*float(point[0]) + coeff[1]*float(point[1]) + coeff[2]
	den = sqrtf(coeff[0]*coeff[0] + coeff[1]*coeff[1])
	if num < 0:
		num = -num
	if den == 0:
		d = float('inf')
	else:
		d = num / den
	if d < T:
		return 1
	return 0

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef np.ndarray[DTYPE_INT_t, ndim=1] cverfyPoints(np.ndarray[DTYPE_FLOAT_t,ndim=1] coeff, np.ndarray[DTYPE_INT_t,ndim=2] points, float T):
	cdef int n_point = points.shape[0]
	cdef np.ndarray[DTYPE_INT_t, ndim=1] array = np.zeros(n_point, dtype=DTYPE_INT)
	cdef int i

	for i in range(n_point):
		array[i] = cverifyPoint(coeff, points[i], T)
	return array

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef void cverfyPoints_pointer(np.ndarray[DTYPE_FLOAT_t,ndim=1] coeff, np.ndarray[DTYPE_INT_t,ndim=2] points, float T, np.ndarray[DTYPE_INT_t, ndim=1] out):
	cdef int n_point = points.shape[0]
	cdef int i

	for i in range(n_point):
		if cverifyPoint(coeff, points[i], T) == 1:
			out[i] = 1
		else:
			out[i] = 0

cdef void cfitLine2Point(np.ndarray[DTYPE_INT_t,ndim=2] point, np.ndarray[DTYPE_FLOAT_t,ndim=1] out):
	cdef float diffX = float(point[1,0] - point[0,0])
	cdef float diffY = float(point[1,1] - point[0,1])
	cdef float c
	c = -diffX*float(point[0,1]) + diffY*float(point[0,0])
	out[1] = diffX
	out[0] = -diffY
	out[2] = c

cdef void cfitLine3Point(np.ndarray[DTYPE_INT_t,ndim=2] point, np.ndarray[DTYPE_FLOAT_t,ndim=1] out):
	cdef np.ndarray[DTYPE_INT_t,ndim=1] x = point[:,0]
	cdef np.ndarray[DTYPE_INT_t,ndim=1] y = point[:,1]
	cdef np.ndarray[DTYPE_INT_t,ndim=2] A = np.vstack([x, np.ones(x.shape[0],dtype=DTYPE_INT)]).T
	cdef float m,c
	m,c = np.linalg.lstsq(A, y)[0]
	out[0] = float(m)
	out[1] = -1
	out[2] = float(c)

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef np.ndarray[DTYPE_INT_t, ndim = 1] crandomRansacSample(int n_sample, int max_, int min_):
	cdef np.ndarray[DTYPE_INT_t, ndim = 1] out = np.zeros(n_sample, dtype=DTYPE_INT)
	cdef int i,val

	for i in range(n_sample):
		val = (rand()%(max_-min_)) + min_
		while ccheckInArrayInt(out, 0, i, val):
			val = (rand()%(max_-min_)) + min_
		out[i] = val
	return out

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
@cython.cdivision(True)
cdef void crandomRansacSample_pointer(int n_sample, int max_, int min_, np.ndarray[DTYPE_INT_t, ndim = 1] out):
	cdef int i,val

	for i in range(n_sample):
		val = (rand()%(max_-min_)) + min_
		while ccheckInArrayInt(out, 0, i, val):
			val = (rand()%(max_-min_)) + min_
		out[i] = val

cdef int ccheckInArrayInt(np.ndarray[DTYPE_INT_t, ndim=1] array, int start, int stop, int val):
	cdef int i
	for i in range(start, stop):
		if val == array[i]:
			return 1
	return 0

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef int ccountNoneZeros(np.ndarray[DTYPE_INT_t,ndim=1] array):
	cdef int count=0,i
	cdef int len_array = array.shape[0]
	for i in range(len_array):
		if array[i] > 0:
			count += 1
	return count

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef void copyNumpyArray(np.ndarray[DTYPE_INT_t, ndim=1] input_, np.ndarray[DTYPE_INT_t, ndim=1] output_, int len_):
	cdef int i
	for i in range(len_):
		output_[i] = input_[i]

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef np.ndarray[DTYPE_INT_t, ndim=1] cransac(np.ndarray[DTYPE_INT_t,ndim=2] points,int maxIter, float T):
	cdef int maxVote = -1
	cdef int len_point = points.shape[0]
	cdef np.ndarray[DTYPE_FLOAT_t, ndim=1] c = np.zeros(3, dtype=DTYPE_FLOAT)
	cdef np.ndarray[DTYPE_INT_t, ndim=1] bestsupporter = np.zeros(len_point, dtype=DTYPE_INT)
	cdef np.ndarray[DTYPE_INT_t, ndim=1] supporter = np.zeros(len_point, dtype=DTYPE_INT)
	cdef np.ndarray[DTYPE_INT_t, ndim=1] sample = np.zeros(2, dtype=DTYPE_INT)
	cdef int i, vote
	for i in range(maxIter):
		crandomRansacSample_pointer(2, len_point-1, 0, sample)
		cfitLine2Point(points[sample,:], c)
		cverfyPoints_pointer(c, points, T, supporter)

		vote = ccountNoneZeros(supporter)
		if vote > maxVote:
			maxVote = vote
			copyNumpyArray(supporter, bestsupporter, len_point)
	return bestsupporter

cpdef verifyPoint(np.ndarray[DTYPE_FLOAT_t,ndim=1] coeff,np.ndarray[DTYPE_INT_t,ndim=1] point, float T):
	return cverifyPoint(coeff, point, T)

cpdef fitLine2Point(np.ndarray[DTYPE_INT_t,ndim=2] point, np.ndarray[DTYPE_FLOAT_t,ndim=1] out):
	cfitLine2Point(point, out)

cpdef np.ndarray[DTYPE_INT_t, ndim=1] verfyPoints(np.ndarray[DTYPE_FLOAT_t,ndim=1] coeff, np.ndarray[DTYPE_INT_t,ndim=2] points, float T):
	return cverfyPoints(coeff, points, T)

cpdef np.ndarray[DTYPE_INT_t, ndim = 1] randomRansacSample(int n_sample, int max_, int min_):
	return crandomRansacSample(n_sample, max_, min_)

cpdef np.ndarray[DTYPE_INT_t, ndim=1] ransac(np.ndarray[DTYPE_INT_t,ndim=2] points,int maxIter, float T):
	return cransac(points, maxIter, T)