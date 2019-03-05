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

	def __init__( self, modelPathStr, positiveThreshold, winSize = ( 40, 40 ), blockSize = ( 8, 8 ), blockStride = ( 4, 4 ),
	   		    cellSize = ( 4, 4 ), nBins = 9, rectangleThreshold = 20, boundingBoxSize = 10 ):
 
		#   get model for classifier
		self.model = loadModel( modelPathStr )

		#	set probability to true
		print self.model.probability

		#	define hog descriptor instance
		self.hogDescriptor = cv2.HOGDescriptor( winSize, blockSize, blockStride, cellSize, nBins )

		#	define bounding box instance
		self.boundingBoxListObject = BoundingBoxList( rectangleThreshold, boundingBoxSize )

		#	flag to check predictor is predict or not yet
		self.isPredict = False

		#	flag to check that have any bounding box
		self.ableToExtraction = True
		
		#	set threshold for positive classification
		self.positiveThreshold = positiveThreshold
		
		#	initial best bounding box attribute
		self.bestBoundingBox = None

	def extractFeature( self, image, whiteContours, objectPointLocation = 'center' ):

		#	get bounding box from white contours first
		self.boundingBoxListObject.getBoundingBox( image, whiteContours, objectPointLocation = objectPointLocation )

		#	set flag to false when didn't get any bounding box
		if len( self.boundingBoxListObject.boundingBoxList ) == 0:
			
			self.ableToExtraction = False
			
			return False

		#	compute feature vector
		for boundingBox in self.boundingBoxListObject.boundingBoxList:

			#	compute!		
			hogFeature = self.hogDescriptor.compute( boundingBox.roiImage )

			#	get feature vector
			boundingBox.featureVector = hogFeature.T

		#	set true when extract feature finish
		self.ableToExtraction = True
		
		return True

	def predict( self ):
		
		#	skip when bounding box not found
		if not self.ableToExtraction:
			self.isPredict = True
			return

		#	loop every bounding list
		for boundingObject in self.boundingBoxListObject.boundingBoxList:

			#	get feature of bounding box
			hogFeature = boundingObject.featureVector

			#	predict by svm, [ 0, 1 ] is index which positive side
			classificationScore = self.model.predict( hogFeature )

			#	get score in form probability
			boundingObject.footballProbabilityScore =  self.model.predict_proba( hogFeature )[ 0, 1 ]

			#	store the result to same object
			if classificationScore[ 0 ] == 1:
				boundingObject.isFootball = True
			else:
				boundingObject.isFootball = False

		#	set flag to true
		self.isPredict = True

	def getBestRegion( self ):
		"""
			
			return None or tuple position
			
		"""

		#	assert for trap, should call after HOG_SVM.predict()
		assert( self.isPredict is True )

		#	initial best position parameter
		bestPosition = None
				
		#	flag to check that at least, one bounding box is football
		foundBall = False
		
		#	re-initial bounding box object
		self.bestBoundingBox = None

		#	calculate
		calculateStatus = self.boundingBoxListObject.calculateDistanceFromPreviousBoundingBox()
		
		
		if calculateStatus == True:
				
			#	sort first by distance
			sortedBoundingBoxList = sorted( self.boundingBoxListObject.boundingBoxList, key = lambda boundingObj : boundingObj.distanceFromPreviousPosition )

			#print sortedBoundingBoxList

			#	loop in sorted list and check from minimum distance that svm is valid, if not get next candidate
			for boundingBoxObj in sortedBoundingBoxList:

				#	check this is a ball ?
				#	NOTE:
				#		I change to check ball with probability
				if boundingBoxObj.footballProbabilityScore > self.positiveThreshold :

					#	save it
					self.boundingBoxListObject.setPreviousBoundingBox( boundingBoxObj )
					
					#	get best bounding box
					self.bestBoundingBox = boundingBoxObj
					
					#	get position
					bestPosition = boundingBoxObj.object2DPosTuple
					
					#	found ball is true
					foundBall = True
					
					#	break the loop
					break
					
		else:
			
			#	get best bounding box from sorted list by probability score
			self.bestBoundingBox = self.boundingBoxListObject.boundingBoxList[ 0 ]
			
			#	get position
			bestPosition = self.bestBoundingBox.object2DPosTuple
			
			#	set previous bounding box
			self.boundingBoxListObject.setPreviousBoundingBox( self.bestBoundingBox )
			
			foundBall = True
			
		#	clear bounding box list
		self.boundingBoxListObject.clearBoundingBoxList()

		#	set back for next iteration
		self.isPredict = False
		
		if foundBall == True:
			return list( bestPosition )
		else:
			return list()
			
	
	def getBestBoundingBox( self ):
		"""
			Must call after get best region
		"""
		
		return self.bestBoundingBox	
	

	
