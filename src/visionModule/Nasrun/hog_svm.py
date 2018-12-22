#!/usr/bin/env python
#
# Copyright (C) 2018  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os

########################################################
#
#	LOCAL IMPORTS
#

import cv2
import numpy as np

import pickle

from region import BoundingBoxList

########################################################
#
#	GLOBALS
#

########################################################
#
#	EXCEPTION DEFINITIONS
#

########################################################
#
#	HELPER FUNCTIONS
#

def loadModel( modelPathStr ):

	with open( modelPathStr, 'r' ) as modelPickle:
		model = pickle.load( modelPickle )

	return model

########################################################
#
#	CLASS DEFINITIONS
#

class HOG_SVM( object ):

	def __init__( self, modelPathStr, winSize = ( 40, 40 ), blockSize = ( 8, 8 ), blockStride = ( 4, 4 ),
						cellSize = ( 4, 4 ), nBins = 9, rectangleThreshold = 20, boundingBoxSize = 10 ):
 
		#   get model for classifier
		self.model = loadModel( modelPathStr )

		#	define hog descriptor instance
		self.hogDescriptor = cv2.HOGDescriptor( winSize, blockSize, blockStride, cellSize, nBins )

		#	define bounding box instance
		self.boundingBoxListObject = BoundingBoxList( rectangleThreshold, boundingBoxSize )

		#	flag to check predictor is predict or not yet
		self.isPredict = False

		#	flag to check that have any bounding box
		self.ableToExtraction = True

	def extractFeature( self, image, whiteContours, objectPointLocation = 'center' ):

		#	get bounding box from white contours first
		self.boundingBoxListObject.getBoundingBox( image, whiteContours, objectPointLocation = objectPointLocation )

		#	set flag to false when didn't get any bounding box
		if len( self.boundingBoxListObject.boundingBoxList ) == 0:
			self.ableToExtraction = False
			return

		#	compute feature vector
		for boundingBox in self.boundingBoxListObject.boundingBoxList:

			#	compute!		
			hogFeature = self.hogDescriptor.compute( boundingBox.roiImage )

			#	get feature vector
			boundingBox.featureVector = hogFeature.T

		#	set true when extract feature finish
		self.ableToExtraction = True

	def predict( self ):
		
		#	skip when bounding box not found
		if not self.ableToExtraction:
			self.isPredict = True
			return

		#	loop every bounding list
		for boundingObject in self.boundingBoxListObject.boundingBoxList:

			#	get feature of bounding box
			hogFeature = boundingObject.featureVector

			#	predict by svm
			classificationScore = self.model.predict( hogFeature )

			#	store the result to same object
			if classificationScore[ 0 ] == 1:
				boundingObject.isFootball = True
			else:
				boundingObject.isFootball = False

		#	set flag to true
		self.isPredict = True

	def chooseBestRegion( self ):

		#	assert for trap, should call after HOG_SVM.predict()
		assert( self.isPredict is True )

		#	skip when bounding box not found
		if not self.ableToExtraction:
			return

		#	calculate
		self.boundingBoxListObject.calculateDistanceFromPreviousBoundingBox()

		#	sort first
		sortedBoundingBoxList = sorted( self.boundingBoxListObject.boundingBoxList, key = lambda boundingObj : boundingObj.distanceFromPreviousPosition )

		#	loop in sorted list and check from minimum distance that svm is valid, if not get next candidate
		for boundingBoxObj in sortedBoundingBoxList:

			#	check this is a ball ?
			if boundingBoxObj.isFootball:
				
				#	save it
				self.boundingBoxListObject.previousBoundingBox = boundingBoxObj

				#	clear list
				self.boundingBoxListObject.clearBoundingBoxList()

				#	set back for next iteration
				self.isPredict = False

				#	return to terminate loop
				return
		
		#	if svm not valid, get first index 
		self.boundingBoxListObject.previousBoundingBox = sortedBoundingBoxList[ 0 ]

		#	clear bounding box list
		self.boundingBoxListObject.clearBoundingBoxList()

		#	set back for next iteration
		self.isPredict = False


		

			


		
	

	