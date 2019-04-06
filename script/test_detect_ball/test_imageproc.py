#!/usr/bin/env python

import os
import sys

import cv2
import numpy as np
#

from scanLine2 import findBoundary

from test_svm_detector import getColorListFromConfig
from colorSegmentation import createColorDefFromDict, colorSegmentation

def nothing( x ):
	pass  

#	user define
pathConfig = '/tmpfs/test.ini'


##	set image path
#imageTestPathStr = '/home/neverholiday/work/ball_detector/raw_data/fibo_field_ball_4/'
#oneImageTestPathStr = os.path.join( imageTestPathStr, 'frame0360.jpg' )
#
###	get image list
##imagePathList = getImageList( imageTestPathStr )
##
###	generate list of image
##imageList = map( cv2.imread, imagePathList )
#
##	initial index indicator
#indexIndicator = 0

#
#	image processing zone
#
cv2.namedWindow( 'originalImage', cv2.WINDOW_NORMAL )
cv2.namedWindow( 'edge', cv2.WINDOW_NORMAL )

cv2.createTrackbar( 'Min', 'edge', 0, 200, nothing )
cv2.createTrackbar( 'Max', 'edge', 0, 200, nothing )

cap = cv2.VideoCapture( 1 )

#	get config
#colorDict = readConfig( 'color_2.ini' )
colorList = getColorListFromConfig( pathConfig )

#	create colordef msg
colorDef = createColorDefFromDict( colorList )


while True:
	
	#img = imageList[ indexIndicator ]
	ret, img = cap.read()
	
	#blur = cv2.bilateralFilter( img, 9, 75, 75 )
	blurGaussian = cv2.GaussianBlur( img, ( 5, 5 ), 0 )
	
	#	get marker
	marker = colorSegmentation( blurGaussian, colorDef )
	
	#	get field boundary
	fieldBoundary, fieldMask = findBoundary( marker, 2, flip = False )
	fieldMask *= 255
	
	#	white marker to white mask
	whiteImageObject = np.zeros( marker.shape )
	whiteImageObject[ marker == 5 ] = 1
	
	#	and operation with field mask
	fieldOnly = cv2.bitwise_and( blurGaussian, blurGaussian, mask = fieldMask )
	whiteObject = ( fieldMask * whiteImageObject ).astype( np.uint8 )
	
	minVal = cv2.getTrackbarPos( 'Min', 'edge' )
	maxVal = cv2.getTrackbarPos( 'Max', 'edge' )
	
	#edgeNormal = cv2.Canny( img, 100, 200 )
	edgeGaussian = cv2.Canny( fieldOnly, 100, 200 )
	
	contours = cv2.findContours(edgeGaussian,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[ 1 ]
	whiteObjectContours = cv2.findContours(whiteObject,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[ 1 ]
	
	#	filter na ja
#	filteredContour = list( filter( lambda x : cv2.contourArea( x ) > 100, contours ) )
#	filteredContour_2 = list( filter( lambda x : cv2.contourArea( x ) > 100, whiteObjectContours ) )
	
	#	draw contour
	#cv2.drawContours(img, filteredContour, -1, (0,0,255), 1)
	
	for cnt in contours:
		
		x, y, w, h = cv2.boundingRect( cnt )
		
		if 20 < w < 200 and 20 < h < 200:
			cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
	
	for cnt in whiteObjectContours:
		
		x, y, w, h = cv2.boundingRect( cnt )
		
		if 20 < w < 200 and 20 < h < 200:
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)	
	

	#	visualize zone
	#	stack
	#edges = np.hstack( ( edgeNormal, edgeGaussian ) )
	
	#	draw field boundary
	cv2.drawContours( img, [ fieldBoundary ], 0, ( 128, 0, 255 ), 1 )
	
	cv2.imshow( 'edge', edgeGaussian )
	cv2.imshow( 'originalImage', img )
	cv2.imshow( 'field mask', fieldOnly )
	cv2.imshow( 'whiteImageObject', whiteObject )
	k = cv2.waitKey( 1 )
	
	if k == ord( 'q' ):
		break
		

cv2.destroyAllWindows()
