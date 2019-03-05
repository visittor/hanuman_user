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

#	GLOBAL, check carefully
OutlierThreshold = 100
ConfigFilePath = '/home/neverholiday/work/ros_ws/src/hanuman_user/config/all_config/robot_config_1.ini'

#	open camera, external camera only
cap = cv2.VideoCapture( 1 )

#	get color config
colorConfigList = configobj.ConfigObj( ConfigFilePath )[ 'ColorDefinitions' ]   
colorConfigList = colorConfigList.values()

#	create color definition msg
colorDef = createColorDefFromDict( colorConfigList ) 

flag = True

while True:
	
	if flag:	
		for i in range( 10 ):
			startTime = time.time()
			#	read
			ret, img = cap.read()

			print "Used time {}".format( time.time() - startTime )
			
		flag = False
	
	startTime = time.time()
	ret, img = cap.read()
	print "Used time {}".format( time.time() - startTime )
	#	filering
	blurImage = cv2.GaussianBlur( img, ( 5, 5 ), 0 )
	
	#	get marker
	marker = colorSegmentation( blurImage, colorDef )
	
	#	get field
	fieldContour, fieldMask = findBoundary( marker, 2, flip = False )
	
	#	get regressor list
	regressorList = findLinearEqOfFieldBoundary( fieldContour )
	
	for regressor in regressorList:
		
		X = np.arange( regressor[ 2 ], regressor[ 3 ] )
		Y = ( regressor[ 0 ] * X ) + regressor[ 1 ]
		
		floorY = np.floor( Y ).astype( X.dtype )
		
		contour = np.vstack( ( X, floorY ) ).transpose()
		contour = contour.reshape( -1, 1, 2 )
		
		cv2.drawContours( img, [ contour ], 0, ( 0, 0, 255 ), 3 )
	
	cv2.drawContours( img, [ fieldContour ], 0, ( 255, 0, 0 ), 1 )
		
	
	#	visualize image
	cv2.imshow( "img", img )
	k = cv2.waitKey( 1 )
	if k == ord( 'q' ):
		break

cap.release()
cv2.destroyAllWindows() 

