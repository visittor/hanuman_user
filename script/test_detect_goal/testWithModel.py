#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

VERSIONNUMBER = 'v1.0'
PROGRAM_DESCRIPTION = "Use with model"

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os

import optparse

import pickle

########################################################
#
#	LOCAL IMPORTS
#

import cv2
import numpy as np

from testBoundaryScanLine import readConfig, colorSegmentation, waterShed, renderColor

from scanLine2 import findBoundary, findNewLineFromRansac

from imageProcessingModule import findGoal

########################################################
#
#	Standard globals
#
NUM_REQUIRE_ARGUMENT = 2

DefaultConfigPath = '/home/neverholiday/work/ros_ws/src/hanuman_user/config/all_config/robot_config_1.ini'

winSize = ( 40, 40 )
blockSize = ( 8, 8 )
blockStride = ( 4, 4 )
cellSize = ( 4, 4 )
nBins = 9
rectangleThreshold = 20
boundingBoxSize = 10

########################################################
#
#	Program specific globals
#

########################################################
#
#	Helper functions
#

def loadPickle( path ):

	with open( path, 'r' ) as f:
		obj = pickle.load( f )
	
	return obj

def createMaskFromContour( cntPoint, shapeOfMask ):
	'''	createMaskFromContours function
	'''

	mask = np.zeros( shapeOfMask, dtype=np.uint8 )

	cv2.drawContours( mask, [cntPoint], 0, 1, -1 )

	return mask

def createFourPointMatrix( x, y, w, h ):
	'''	createFourPointMatrix function
	'''

	#	vector of position four point
	topLeft = np.array( ( x, y ) )
	topRight = np.array( ( x + w, y ) )
	bottomLeft = np.array( ( x, y + h ) )
	bottomRight = np.array( ( x + w, y + h ) )

	#	stack
	fourPointVector = np.vstack( ( bottomLeft, topLeft, topRight, bottomRight ) )

	#	reshape to three dimension
	fourPointMatrix = fourPointVector.reshape( -1, 1, 2 )

	return fourPointMatrix

def checkPointThatInsideContours( contourList, point ):
	'''	checkPointThatInsideContours function
	'''

	#	loop over the contour list
	for contour in contourList:

		#	test result contour list
		testResult = cv2.pointPolygonTest( contour, tuple( point ), measureDist = True )

		if testResult > 1.0:
			return True

	return False

def expandBoundingBox( topLeft_X, topLeft_Y, bottomRight_X, bottomRight_Y, imageWidth, imageHeight, expandSize = 10 ):
	'''	expandBoundingBox function
	'''
	
	#	calculate expand fourpoints
	newTopLeft_X = max( 0, topLeft_X - expandSize )
	newTopLeft_Y = max( 0, topLeft_Y - expandSize )
	newBottomRight_X = min( imageWidth, bottomRight_X + expandSize )
	newBottomRight_Y = min( imageHeight, bottomRight_Y + expandSize )

	return newTopLeft_X, newTopLeft_Y, newBottomRight_X, newBottomRight_Y



########################################################
#
#	Class definitions
#

########################################################
#
#	Function bodies
#

########################################################
#
#	main
#	
def main():
	
	#	define usage of programing
	programUsage = "python %prog {} [option]".format( '[pathModel] [testVideo]' ) + str( VERSIONNUMBER ) + ', Copyright (C) 2019 FIBO/KMUTT'

	#	initial parser instance
	parser = optparse.OptionParser( usage = programUsage, description=PROGRAM_DESCRIPTION )

	parser.add_option( '--configPath', dest='configPath', action='store', type='string',
						default=DefaultConfigPath, help='Specify config path' )

	#	add option
	( options, args ) = parser.parse_args()

	#	check number of argument from NUM_REQUIRE_ARGUMENT
	if len( args ) != NUM_REQUIRE_ARGUMENT:	
		
		#	raise error from parser
		parser.error( "require {} argument(s)".format( NUM_REQUIRE_ARGUMENT ) )
	

	#########################################################
	#
	#		get option and argument
	#
	pathModel = args[ 0 ]
	pathTestVideo = args[ 1 ]

	#	get option
	configPath = options.configPath

	#	read config
	colorDefList = readConfig( configPath )

	print "\nLet test this code !\n"

	print "Testing information"
	print "	Path model : {}".format( pathModel )
	print "	Path of testing video : {}".format( pathTestVideo )

	cv2.namedWindow( 'frame', cv2.WINDOW_NORMAL ) 
	cv2.namedWindow( 'rendered_color', cv2.WINDOW_NORMAL )
	cv2.namedWindow( 'mask_field', cv2.WINDOW_NORMAL )

	# cap = cv2.VideoCapture( pathTestVideo )
	cap = cv2.VideoCapture( 1 )

	hog = cv2.HOGDescriptor( winSize, blockSize, blockStride, cellSize, nBins )
	model = loadPickle( pathModel )

	#	get number of frames
	# numFrames = cap.get( cv2.CAP_PROP_FRAME_COUNT ) 

	#	initial index frame counter
	frameIdx = 0

	#	create list to conatin bounding box point
	boundingBoxList = list()

	while True:

		#	retrieve from camera
		ret, frame = cap.read()

		#	get image property
		imageWidth = frame.shape[ 1 ]
		imageHeight = frame.shape[ 0 ]

		#	set frame index
		# cap.set( cv2.CAP_PROP_POS_FRAMES, frameIdx )

		#	convert to gray scale
		imageGray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )

		#	Blur it!!!
		frame = cv2.GaussianBlur( frame, ( 5, 5 ), 0 )

		#	segment!!!
		marker = colorSegmentation( frame, colorDefList )
		marker = waterShed( frame, marker )

		#	get field contour and mask of field
		fieldContour, fieldMask = findBoundary( marker, 2 )

		fieldMask *= 255

		#	get new contour
		ransacContours = findNewLineFromRansac( fieldContour, imageWidth, imageHeight )

		newFieldMask = createMaskFromContour( ransacContours, marker.shape )

		whiteObject = np.zeros( marker.shape, dtype = np.uint8 )
		whiteObject[ marker == 5 ] = 1
	
		#	new white object in field
		whiteObjectInfield = whiteObject * newFieldMask * 255

		#	test opening
		kernel = np.ones( (20,5), dtype=np.uint8 )
		whiteObjectInfield = cv2.morphologyEx( whiteObjectInfield, cv2.MORPH_OPEN, kernel )

		#	find white contours
		whiteContours = cv2.findContours( whiteObjectInfield, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[1]

		for cnt in whiteContours:
			x, y, w, h  = cv2.boundingRect( cnt )

			topLeft = ( x, y )
			bottomRight = ( x+w, y+h )

			#	calculate expand size
			newTopLeft_X, newTopLeft_Y, newBottomRight_X, newBottomRight_Y = expandBoundingBox( topLeft[0], topLeft[1], 
																								bottomRight[0], bottomRight[1], 
																								imageWidth, imageHeight, 
																								expandSize=10 )

			#	ROI image
			roiImage = imageGray[ newTopLeft_Y : newBottomRight_Y, newTopLeft_X : newBottomRight_X ].copy()

			#	rescale image
			roiImage = cv2.resize( roiImage, ( 40, 40 ) )

			#	compute hog
			featureVector = hog.compute( roiImage ).T

			#	predict
			score = model.predict( featureVector )

			probabilityScore = model.predict_proba( featureVector )[ 0, 1 ]

			cv2.putText( frame, "{:.3f}".format( probabilityScore ), (newTopLeft_X, newTopLeft_Y), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1, ( 0,0,255 ), 2, cv2.LINE_AA )
 
			if score[0] == 1:

				cv2.rectangle( frame, (newTopLeft_X, newTopLeft_Y), (newBottomRight_X, newBottomRight_Y), ( 0, 255, 0 ), 2 )

			else:

				cv2.rectangle( frame, (newTopLeft_X, newTopLeft_Y), (newBottomRight_X, newBottomRight_Y), ( 0, 0, 255 ), 2 )

			boundingBoxList.append( createFourPointMatrix( x, y, w, h ) )
			
		#	find goal 
		goalPositionList = findGoal( ransacContours, marker )

		if goalPositionList is not None:

			goalPositionArray = np.array( goalPositionList ).reshape( -1, 2 )

			goalPositionArray[ :, 0 ] = goalPositionArray[ :, 0 ] + 20 

			goalPositionList = list( goalPositionArray )

			goalPositionListFiltered = filter( lambda x : checkPointThatInsideContours( boundingBoxList, x ), goalPositionList )

			for goal in goalPositionListFiltered:

				cv2.circle( frame, tuple( goal ), 5, ( 0, 0, 255 ), -1 )

			#	Offset for debug
			#	REMOVE LATER
			# goalPositionArray[ :, 1 ] = goalPositionArray[ :, 1 ] + 20
			# for goal in list( goalPositionArray ):

			# 	cv2.circle( frame, tuple( goal ), 5, ( 255, 0, 0 ), -1 )
			

		# 
		#	Visualize zone 
		# 	

		#	get render image
		renderImage = renderColor( marker, colorDefList )
		
		cv2.drawContours( frame, [ ransacContours ], 0, ( 255, 0, 0 ), 2 )
		#cv2.drawContours( frame, whiteContours, -1, ( 0, 255, 0 ), 2 )

		cv2.imshow( 'frame', frame )
		cv2.imshow( 'rendered_color', renderImage )
		cv2.imshow( 'mask_field', whiteObjectInfield )
		k = cv2.waitKey( 24 )
		if k == ord( 'q' ):
			break
		# elif k == ord( 'd' ):
		# 	if frameIdx <= numFrames:
		# 		frameIdx += 1

		# elif k == ord( 'a' ):
		# 	if frameIdx > 0:
		# 		frameIdx -= 1

		del boundingBoxList[ : ]
 
	cap.release()
	cv2.destroyAllWindows()

########################################################
#
#	call main
#

if __name__=='__main__':
	main()

