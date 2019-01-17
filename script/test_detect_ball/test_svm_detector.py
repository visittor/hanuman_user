#!/usr/bin/env python
#
# Copyright (C) 2018  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

VERSIONNUMBER = 'v1.0'
PROGRAM_DESCRIPTION = "Test svm with watershed"

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os

import optparse


########################################################
#
#	LOCAL IMPORTS
#

import pickle

import cv2
import numpy as np

#from test_watershed import colorSegmentation, readConfig

from scanLine2 import findBoundary
from visionModule.Nasrun.hog_svm import HOG_SVM
from colorSegmentation import createColorDefFromDict, colorSegmentation

import configobj

########################################################
#
#	Standard globals
#
NUM_REQUIRE_ARGUMENT = 2

########################################################
#
#	Program specific globals
#

########################################################
#
#	Helper functions
#

def getImageList( pathFrameStr ):
	"""
	get list of frame image directory
	"""
	
	#	get abs path
	absFramePathStr = os.path.abspath( pathFrameStr )

	#	get list of image directory
	absImagePathList = os.listdir( absFramePathStr )

	#	sort first
	absImagePathList.sort()

	#	bring abs path to this directory and add name of each image
	absImagePathList = map( lambda imageName : absFramePathStr + '/' + imageName, absImagePathList )

	return absImagePathList

def loadModel( modelPathStr ):

	with open( modelPathStr, 'r' ) as modelPickle:
		model = pickle.load( modelPickle )

	return model

def getColorListFromConfig( colorConfigPathStr ):
	
	#	initial configobj
	colorConfig = configobj.ConfigObj( colorConfigPathStr )

	#	get color list from object of color config
	colorList = colorConfig.values()

	return colorList

def openingMorphologicalWithCircularMask( binaryImage ):

	#	create mask
	circularMask = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 15, 15 ) )

	#	dilate and erode = opening method
	erodeImage = cv2.erode( binaryImage, circularMask )
	dilateImage = cv2.dilate( erodeImage, circularMask )

	return dilateImage.copy()

def expandAreaBoundingBox( pixelSize, boundingBox, imgWidth, imgHeight ):
	
	#	get top-left position and bottom-right position from bounding box
	#	top-left : (x1, y1)
	x1 = boundingBox[ 0 ]
	y1 = boundingBox[ 1 ]
	
	#	bottom-right : (x2,y2)	
	x2 = x1 + boundingBox[ 2 ]
	y2 = y1 + boundingBox[ 3 ]

	#	actual bounding box
	x1Actual = max( 0, x1 - pixelSize )
	y1Actual = max( 0, y1 - pixelSize )
	x2Actual = min( imgWidth, x2 + pixelSize )
	y2Actual = min( imgHeight, y2 + pixelSize )

	return x1Actual, y1Actual, x2Actual, y2Actual

def extractFeatureHog( image ):

	#	static parameter from training
	winSize = ( 40, 40 )
	blockSize = ( 8, 8 )
	blockStride = ( 4, 4 )
	cellSize = ( 4, 4 )
	nBins = 9

	#	instance of HOG descriptor feature
	hogDescriptor = cv2.HOGDescriptor( winSize, blockSize, blockStride, cellSize, nBins )

	#	get vector of HOG
	feature = hogDescriptor.compute( image )

	return feature

# def isModelValid( model, sample, data = None, residual_threshold = 0, mask = None ):
# 	sample_model_residuals = np.abs(model.residuals( data ))
#     # consensus set / inliers
# 	sample_model_inliers = data[sample_model_residuals < residual_threshold]

# 	center = ( int( model.params[0]), int( model.params[1] ) )
# 	r = model.params[2]

# 	cntArea = cv2.contourArea( sample_model_inliers.reshape(-1,1,2).astype( np.int32 ) )
# 	cirArea = np.pi * model.params[2] ** 2

# 	maskCir = np.zeros(mask.shape, np.uint8)

# 	cv2.circle( maskCir, center, int(r), 255, -1 )
# 	mean = cv2.mean( mask, mask = maskCir )[0] / 255.0

# 	# cv2.imshow( 'eie', mask & maskCir )
# 	# cv2.waitKey( 1 )
# 	print mean
# 	if 0.55 <= mean <= 1.5:
# 		print mean
# 		print "True"
# 		return True
	
# 	return False


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
	programUsage = "python %prog arg [option] " + str( VERSIONNUMBER ) + ', Copyright (C) 2018 FIBO/KMUTT'

	#	initial parser instance
	parser = optparse.OptionParser( usage = programUsage, description=PROGRAM_DESCRIPTION )

	#	add option of main script
	parser.add_option( "--colorConfig", dest = "colorConfig", action = 'store', type = 'string',
						help = "Specify color config.", default = '../../config/color_config/colordef_test.ini' )

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

	framePathStr = args[ 0 ]
	modelPathStr = args[ 1 ]

	colorConfigPathStr = options.colorConfig

	#	get abs path
	frameAbsPathStr = getImageList( framePathStr )

	#	get config
	#colorDict = readConfig( 'color_2.ini' )
	colorList = getColorListFromConfig( colorConfigPathStr )

	#	create colordef msg
	colorDef = createColorDefFromDict( colorList )

	#	load image sequence
	frameList = map( cv2.imread, frameAbsPathStr )

	#	initial index frame
	idxFrameInt = 0

	#	get model
	model = loadModel( modelPathStr )

	#	initial hog/svm instance for classifier
	predictor = HOG_SVM( modelPathStr )

	while True:

		#	get image
		img = frameList[ idxFrameInt ]

		#	blur image
		blurImage = cv2.blur( img, ( 5, 5 ) )

		#	get marker
		marker = colorSegmentation( blurImage, colorDef )

		#	get only white object from marker
		#	NOTE :
		#		Change id of white marker, follow chattarin colordef template
		#		ID White : 5, old : 8
		#		ID Green : 2, old : 1
		whiteImageObject = np.zeros( marker.shape )
		whiteImageObject[ marker == 5 ] = 1
 
		#	get field boundary
		fieldBoundary, fieldMask = findBoundary( marker, 2, flip = False )
		
		#	get mask only ball
		whiteObjectMask = fieldMask * whiteImageObject 
		whiteObjectMask *= 255

		#	change back to uint8 (opencv support only uint8)
		whiteObjectMask = whiteObjectMask.astype( np.uint8 )

		#	get contours from white object
		whiteContours = cv2.findContours( whiteObjectMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

		#	copy image for visualize
		visualizeImage = img.copy()

		#	extract feature all white feature
		predictor.extractFeature( img, whiteContours, objectPointLocation = 'bottom' )

		#	predict all white feature, which the ball is ?
		predictor.predict()

		# #	get bounding box list from cv2.boundingRect
		# boundingBoxList = map( cv2.boundingRect, whiteContours )

		# #	filter
		# filterFunction = lambda boundingBoxTuple : boundingBoxTuple[ 2 ] >= 10 and boundingBoxTuple[ 3 ] >= 10
		# boundingBoxFilterdList = filter( filterFunction, boundingBoxList )

		# #	filter again check rectangle
		# filterNonRectFunction = lambda boundingBoxTuple : abs( boundingBoxTuple[ 2 ] - boundingBoxTuple[ 3 ] ) <= 20
		# boundingBoxRectList = filter( filterNonRectFunction, boundingBoxFilterdList )

		# print len( boundingBoxRectList )

		# for boundingBox in boundingBoxRectList:

		# 	x1Actual, y1Actual, x2Actual, y2Actual = expandAreaBoundingBox( 10, boundingBox, img.shape[ 1 ], img.shape[ 0 ] )

		# 	# cv2.rectangle( visualizeImage, ( x, y ), 
		# 	# 							   ( x + width, y + height ), ( 255, 0, 0 ), 2 )

		# 	cv2.rectangle( visualizeImage, ( x1Actual, y1Actual ), 
		# 								   ( x2Actual, y2Actual ), ( 0, 255, 255 ), 2 )

		# 	#	get roi
		# 	roiCandidateImage = img[ y1Actual : y2Actual, x1Actual : x2Actual ].copy()
			
		# 	#	resize
		# 	roiCandidateResizedImage = cv2.resize( roiCandidateImage, ( 40, 40 ) )

		# 	#	extract feature
		# 	featureVector = extractFeatureHog( roiCandidateResizedImage ).T
			
		# 	#	predict
		# 	predictClass = model.predict( featureVector )

		# 	if predictClass[ 0 ] == 1:
		# 		cv2.circle( visualizeImage, ( boundingBox[ 0 ] + boundingBox[ 3 ] / 2, boundingBox[ 1 ] + boundingBox[ 3 ] / 2 ), 
		# 								      boundingBox[ 3 ] , ( 0, 255, 0 ), 2 )
				
		#
		#	visualization zone
		#

		for boundingBoxObject in predictor.boundingBoxListObject.boundingBoxList:

			# cv2.rectangle( visualizeImage, boundingBoxObject.topLeftPositionTuple, 
			# 			   boundingBoxObject.bottomRightPositionTuple, ( 255, 0, 0 ), 2 )

			if boundingBoxObject.isFootball is True:
				
				cv2.rectangle( visualizeImage, boundingBoxObject.topLeftPositionTuple, 
						   boundingBoxObject.bottomRightPositionTuple, ( 255, 0, 0 ), 2 )

				cv2.circle( visualizeImage, boundingBoxObject.object2DPosTuple, 5, ( 0, 255, 0 ), -1 )
			
			else:

				cv2.rectangle( visualizeImage, boundingBoxObject.topLeftPositionTuple, 
						   boundingBoxObject.bottomRightPositionTuple, ( 0, 0, 255 ), 2 )
				
				cv2.circle( visualizeImage, boundingBoxObject.object2DPosTuple, 5, ( 0, 255, 0 ), -1 )

		#	get best region
		predictor.chooseBestRegion()
		
		if predictor.boundingBoxListObject.previousBoundingBox is not None:
			cv2.circle( visualizeImage, predictor.boundingBoxListObject.previousBoundingBox.object2DPosTuple, 10, ( 255, 0, 0 ), -1 )
		
		#print idxFrameInt
		predictor.boundingBoxListObject.clearBoundingBoxList()
		#	draw contours
		cv2.drawContours( visualizeImage, [ fieldBoundary ], 0, ( 128, 0, 255 ), 1 )

		#	show image
		cv2.imshow( "show", visualizeImage )
		cv2.imshow( "ball mask", whiteObjectMask )
		
		#	waitkey and break out of the loop
		k = cv2.waitKey( 50 )
		if k == ord( 'q' ):
			break
		elif k == ord( 'd' ):
			idxFrameInt += 1
		elif k == ord( 'a' ):
			idxFrameInt -= 1

		# #	increment index
		# idxFrameInt += 1
		
		# #	reset frame index 
		if idxFrameInt >= len( frameList ):
			idxFrameInt = len( frameList ) - 1
		elif idxFrameInt <= 0:
			idxFrameInt = 0
	
	cv2.destroyAllWindows()




########################################################
#
#	call main
#

if __name__=='__main__':
	main()

