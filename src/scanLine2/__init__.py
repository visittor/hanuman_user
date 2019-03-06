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
	
def findLinearEqOfFieldBoundary( contourPoint, outlierThreshold = 100 ):
	'''	find m, c, x0, xf of field bounndary 
		args:
			contourPoint : contour point from field boundary
		return:
			propertyLineList : list of tuple which contain m, c, x0, xf
	'''
	
	#	reshape contour point
	contourPoint = contourPoint.reshape( -1, 2 )
	
	#	get x, y array
	x = contourPoint[ :, 0 ].reshape( -1, 1 )
	y = contourPoint[ :, 1 ]
	
	#	initial inlier mask and outlier mask
	inlierMask = np.full( y.size, False )
	outlierMask = np.full( y.size, True )
	
	#	initial list of line property
	propertyLineList = list()
	
	#	initial remain of point x and y
	xRemain = x[ outlierMask ]
	yRemain = y[ outlierMask ]
	
	for i in xrange( 2 ):
		
		#	initial regressor instance
		regressor = linear_model.RANSACRegressor( residual_threshold = 3.0 )
	
		#	fit equation
		regressor.fit( xRemain, yRemain )
		
		#	get inlier and outlier mask
		inlierMask = regressor.inlier_mask_
		outlierMask = np.logical_not( inlierMask )
		
		#	get initial position and final position
		# x0 = xRemain[ inlierMask ][ 0 ]
		# xf = xRemain[ inlierMask ][ -1 ]
		x0 = np.array( [xRemain[ inlierMask ].min()] )
		xf = np.array( [xRemain[ inlierMask ].max()] )
	
		y0 = regressor.predict( x0.reshape( -1, 1 ) )
		yf = regressor.predict( xf.reshape( -1, 1 ) )
		
		#	get outlier from mask
		xRemain = xRemain[ outlierMask ]
		yRemain = yRemain[ outlierMask ]
		
		#	calculate m, c
		try:
			#	m = ( yf - y0 ) / ( xf - x0 )
			#	c = m*x0 - y0
			m = ( yf.reshape( -1 ) - y0.reshape( -1 ) ) / ( xf.reshape( -1 ) - x0.reshape( -1 ) )
			c = y0.reshape( -1 ) - ( m.reshape( -1 ) * x0.reshape( -1 ) )
		
			#	list format ( m, c, x0, xf )
			propertyLineList.append( [ m[ 0 ], c[ 0 ], x0[ 0 ], xf[ 0 ] ] )
		
#			print "m : {}".format( m )
#			print "c : {}".format( c ) 
		
		except ZeroDivisionError:
			continue
		
		if len( yRemain ) < outlierThreshold:
			break

	x0 = contourPoint[0,0]
	xf = contourPoint[-1,0]

	if len( propertyLineList ) == 1:
		return [ ( propertyLineList[0][0], propertyLineList[0][1], x0, xf ) ]


	## y1 = m1x1 + c1
	## y1 = m2x1 + c2

	m1, c1, x0_1, xf_1 = propertyLineList[0][:]
	m2, c2, x0_2, xf_2 = propertyLineList[1][:]

	if m1 == m2:
		return [ ( m1, c1, x0, xf ) ]

	intersec_x = ( c1 - c2 ) / ( m2 - m1 )

	if x0_1 < x0_2:
		return [ (m1, c1, x0, intersec_x ), (m2, c2, intersec_x, xf) ]

	else:
		return [ (m2, c2, x0, intersec_x), (m1, c1, intersec_x, xf) ]

def findChangeOfColor( colorMap, color1, color2, mask = None, axis = 0, step = 1, doFlip = False ):
	
	assert axis == 0 or axis == 1, "Only `axis` can only be 0 or 1."

	mask = mask if mask is not None else np.ones( colorMap.shape[:2] )

	marker = colorMap * mask
	# marker[ marker == 255 ] = color2
	marker[ np.logical_and( marker!=color1, marker!=color2 ) ] = np.max( marker ) + 100

	if doFlip:
		marker = np.flip(marker, axis = axis)

	sli = tuple( slice(0,None,1) if i != axis else slice(0,None,2) for i in range(2) ) 

	diffMark = np.diff( marker[sli], axis = axis )
	yEdge, xEdge = np.where( diffMark == color1 - color2 )

	if axis == 0:
		yEdge *= 2

	else:
		xEdge *= 2

	pointClound = []
	for i in range( 0, colorMap.shape[ (axis+1) % 2 ], step ):
		yCoor = yEdge[ xEdge == i ].astype( int )
		yCoor = np.vstack( ( np.ones( yCoor.shape, dtype = int ) * i, yCoor ) ).transpose()

		pointClound.append( yCoor )

	return pointClound
