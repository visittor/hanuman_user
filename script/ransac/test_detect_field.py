#!/usr/bin/env python


import cv2
import numpy as np

import configobj

from colorSegmentation import colorSegmentation, createColorDefFromDict
from scanLine2 import findBoundary

from matplotlib import pyplot as plt

from sklearn import linear_model

import time

from scanLine2 import findLinearEqOfFieldBoundary


OutlierThreshold = 100

#	path of test image
imgTestPath = 'image_test/frame.0004.jpg'

#	config for testing image
configTestPath = 'color_config.ini'

#	define window gui
#cv2.namedWindow( 'image', cv2.WINDOW_NORMAL )
#cv2.namedWindow( 'marker', cv2.WINDOW_NORMAL )

#	read testing image
img = cv2.imread( imgTestPath )
blurImage = cv2.GaussianBlur( img, ( 5, 5 ), 0 )

#	get color config
colorConfigList = configobj.ConfigObj( configTestPath )[ 'ColorDefinitions' ]   
colorConfigList = colorConfigList.values()

#	create color definition msg
colorDef = createColorDefFromDict( colorConfigList ) 

#	get marker
marker = colorSegmentation( blurImage, colorDef )

#	get field
fieldContour, fieldMask = findBoundary( marker, 2, flip = False )

fieldMask = fieldMask.astype( np.uint8 )
fieldMask *= 255

greenObject = np.zeros( ( img.shape[ 0 ], img.shape[ 1 ] ), dtype = np.uint8 )
greenObject[ marker == 2 ] = 255

startTime = time.time()

regressorList = findLinearEqOfFieldBoundary( fieldContour )

pointClound = fieldContour.reshape( -1, 2 )
#
##	split x and y
##x = pointClound[ :, 0 ].reshape( -1, 1 )
x = pointClound[ :, 0 ].reshape( -1, 1 )
y = pointClound[ :, 1 ]

##	initial inlier mask and outlier mask
#inlierMask = np.full( y.size, False )
#outlierMask = np.full( y.size, True )
#
##	calculate mad
#medY = np.median( y )
#madY = np.median( np.abs( y - medY ) )
#
#print "median of y : {}".format( medY )
#print "MAD of y : {}".format( madY )
#
##	initial regressor list
#regressorList = list()
#
#xRemain = x[ outlierMask ]
#yRemain = y[ outlierMask ]
#
#
##	define ransac model
##regressor = linear_model.RANSACRegressor( residual_threshold = 3.0 )
#
#
##while len( yRemain ) > OutlierThreshold:
#
#for i in xrange( 2 ):
#
#	regressor = linear_model.RANSACRegressor( residual_threshold = 3.0 )
#
#	#	using ransac to fit
#	regressor.fit( xRemain, yRemain )
#
#	#	get inlier and outlier
#	inlierMask = regressor.inlier_mask_
#	outlierMask = np.logical_not( inlierMask )
#	
#	print "number of outlier : {}".format( len( yRemain[ outlierMask ] ) )
#	
#	x0 = xRemain[ inlierMask ][ 0 ]
#	xf = xRemain[ inlierMask ][ -1 ]
#	
##	print x0.reshape( -1, 1 )
##	print xf.reshape( -1, 1 )
#	
#	y0 = regressor.predict( x0.reshape( -1, 1 ) )
#	yf = regressor.predict( xf.reshape( -1, 1 ) )
#	
##	print y0.reshape( -1, 1 )
##	print yf.reshape( -1, 1 )
#	
#	#	find next outlier
#	xRemain = xRemain[ outlierMask ]
#	yRemain = yRemain[ outlierMask ]
#	
#	try:
#		m = ( yf.reshape( -1 ) - y0.reshape( -1 ) ) / ( xf.reshape( -1 ) - x0.reshape( -1 ) )
#		c = y0.reshape( -1 ) - ( m.reshape( -1 ) * x0.reshape( -1 ) )
#		
#		#	list format ( m, c, x0, xf )
#		regressorList.append( ( m[ 0 ], c[ 0 ], x0[ 0 ], xf[ 0 ] ) )
#		
#		print "m : {}".format( m )
#		print "c : {}".format( c ) 
#		
#	except ZeroDivisionError:
#		continue
#		
#	if len( yRemain ) < OutlierThreshold:
#		break
#	
#	#regressorList.append( ( ,  )
#	
#print regressorList

stopTime = time.time() - startTime

print "Using time {}".format( stopTime )


plt.subplot( 121 )
plt.xlim( 0, 640 )
plt.ylim( 480, 0 )
#plt.scatter( x, y, marker = '*' )
#plt.plot( x, m*x + c, color = 'red' )
#plt.scatter( x[ inlierMask ], y[ inlierMask ], marker = 'o', color = 'blue' ) 
#plt.scatter( x[ outlierMask ], y[ outlierMask ], marker = 'o', color = 'red' )
plt.scatter( x, y, marker = 'o', color = 'red' )

for reg in regressorList:
	X = np.arange( reg[ 2 ], reg[ 3 ]  )
	Y = ( reg[ 0 ] * X ) +  reg[ 1 ]
#	yPred = reg.predict( x )
	plt.plot( X, Y )


plt.subplot( 122 )
plt.imshow( fieldMask, cmap = 'gray' )

plt.show()

#
#test = marker == 2
#
#print test.any( axis = 0 ).shape


#
#cv2.imshow( 'image', blurImage )
#cv2.imshow( 'marker', fieldMask )
#cv2.waitKey( 0 )
#cv2.destroyAllWindows()
