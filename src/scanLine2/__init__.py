import cv2
import numpy as np 
import time
import sys

from sklearn import linear_model

_RANSAC = linear_model.RANSACRegressor()

def findFirstValue2D( array, val, axis = 0, invalid_val = -1):
	mask = array == val
	return np.where( mask.any( axis = axis ), mask.argmax( axis = axis), invalid_val )

def _strided_app( array, winSize ):  # Window len = L, Stride len/stepsize = S
	nrows = array.size - winSize + 1
	n = array.strides[0]
	return np.lib.stride_tricks.as_strided(array, shape=(nrows,winSize), strides=(n,n))

def findBoundary( colorMap, colorID, flip = False ):
	colorMap = colorMap if not flip else colorMap[::-1]

	y = findFirstValue2D( colorMap, colorID, axis = 0 )
	x = np.arange( colorMap.shape[1] )

	# # mask[-2] = True
	y = np.pad( y, 5, 'edge' )
	y = np.max( _strided_app(y, 11), axis = 1).astype( np.int32 )

	meanY = y.mean()
	varY = np.std( y )
 
	diff = y - meanY
	mask = diff > -1.5* varY
	mask[:2] = True
	mask[-2:] = True
	y = y[mask]
	x = x[mask]

	y[0] = colorMap.shape[0] - 1
	y[-1] = colorMap.shape[0] - 1
	x[0] = 0
	x[-1] = colorMap.shape[1] - 1

	contour = np.vstack( ( x, y ) ).transpose()
	contour = contour.reshape( -1, 1, 2 )

	mask = np.zeros( colorMap.shape, dtype = np.uint8 )
	cv2.drawContours( mask, [ contour ], 0, 1, -1 )

	return contour, mask

def findChangeOfColor( colorMap, color1, color2, mask = None, axis = 0, step = 1, doFlip = False ):
	
	mask = mask if mask is not None else np.ones( colorMap.shape[:2] )

	marker = colorMap * mask
	marker[ marker == 255 ] = color2
	marker[ np.logical_and( marker!=color1, marker!=color2 ) ] = np.max( marker ) + 100

	if doFlip:
		marker = np.flip(marker, axis = axis)

	diffMark = np.diff( marker, axis = axis )
	yEdge, xEdge = np.where( diffMark == color1 - color2 )

	pointClound = []
	for i in range( 0, colorMap.shape[ (axis+1) % 2 ], step ):
		yCoor = yEdge[ xEdge == i ].astype( int )
		yCoor = np.vstack( ( np.ones( yCoor.shape, dtype = int ) * i, yCoor ) ).transpose()

		pointClound.append( yCoor )

	return pointClound
