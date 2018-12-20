#!/usr/bin/env python

import cv2
import numpy as np
import configobj

import os
import sys

from scanLine2 import findBoundary

ROS_WORKSPACE = os.getenv( 'ROS_WORKSPACE' )

def colorSegmentation( img, colorDef ):
	"""
	Find the marker that watershed want
	"""

	imgHSV = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

	marker = np.zeros( ( img.shape[ 0 ], img.shape[ 1 ] ), dtype = np.int32 )
	
	for colorKey in colorDef.keys():
		
		lower = np.array( [ int( colorDef[ colorKey ][ "H_Min" ] ), int( colorDef[ colorKey ][ "S_Min" ] ), int( colorDef[ colorKey ][ "V_Min" ] ) ] )
		upper = np.array( [ int( colorDef[ colorKey ][ "H_Max" ] ), int( colorDef[ colorKey ][ "S_Max" ] ), int( colorDef[ colorKey ][ "V_Max" ] ) ] ) 

		colorMask = cv2.inRange( imgHSV, lower, upper )
		marker[ colorMask != 0 ] = int( colorDef[ colorKey ][ "ID" ] )
	
	return marker

def waterShed( img, marker ):
	"""
	Water shred function (from opencv)
	"""
	colorSeg = cv2.watershed( img, marker )
	return colorSeg

def readConfig( pathConfig ):
	"""
	Read config and get dictionary
	"""
	
	#   create instance of config object
	config = configobj.ConfigObj( pathConfig )

	#	get color definitions
	colorDefinition = config[ 'ColorDefinitions' ]

	return colorDefinition

def renderImage( marker, colorDict ):
	""" Render image for visualize """

	#	get width height of image
	width, height = marker.shape

	#	create blank image
	renderImage = np.zeros( ( width, height, 3 ), dtype = np.uint8 )

	#	loop and set color to render image
	for colorKey in colorDict.keys():

		#	set color
		renderImage[ marker == int( colorDict[ colorKey ][ "ID" ] ) ] = eval( colorDict[ colorKey ][ "RenderColor_RGB" ] )
	
	#	switch channel
	renderImage[ :, :, 0 ], renderImage[ :, :, 2 ] = renderImage[ :, :, 2 ], renderImage[ :, :, 0 ]

	return renderImage

def main():
	
	#	initialize camera
	cap = cv2.VideoCapture( '/home/neverholiday/work/color_calibrator/video/field3.avi' )
	
	#
	#	initialize classifier	
	#
	modelPathStr = ROS_WORKSPACE + '/src/hanuman_user/model/data_haar111217_2.xml'
	classifier = cv2.CascadeClassifier( modelPathStr )

	#	get config ini
	pathConfig = 'color.ini'

	#	get color dictionary
	colorDict = readConfig( pathConfig )

	while True:

		#	capture
		ret, frame = cap.read()

		if ret:
			#	change to gray image
			grayImage = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )

			#	smoothen image
			blurImage = cv2.blur( frame, ( 5, 5 ) )

			#	marker
			marker = colorSegmentation( blurImage, colorDict )

			#	get field boundary
			fieldBoundary, fieldMask = findBoundary( marker, 1, flip = False )

			#	ged rid off outside boundary of field
			fieldImage = cv2.bitwise_and( grayImage, grayImage, mask = fieldMask )

			#	get ball
			footballs = classifier.detectMultiScale( fieldImage, 1.1, 5 )
			for ( x, y, w, h ) in footballs:
				cv2.rectangle( frame, ( x, y ), ( x + w, y + h ), ( 255, 0, 0 ), 2 )

			#print np.var( fieldBoundary[ :, 1 ])
			#	throw marker to water shed
			#markerWaterShed = waterShed( frame, marker )

			#	draw contours
			cv2.drawContours( frame, [ fieldBoundary ], 0, ( 0, 0, 255 ), 3 )

			#	get render image
			colorImage = renderImage( marker, colorDict )

			cv2.imshow( "image", frame )
			cv2.imshow( "segmentation", colorImage )
			cv2.imshow( "mask", fieldMask )

			k = cv2.waitKey( 1 )
			if k == ord( 'q' ):
				break
		else:
			
			cap.set( cv2.CAP_PROP_POS_FRAMES, 0 )

	cap.release()
	cv2.destroyAllWindows()
 
if __name__ == "__main__":
	main()	



