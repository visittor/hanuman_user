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
import math

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
class HOG_predictor( object ):

	def __init__( self, ballThreshold = 0.5, goalThreshold = 0.5, winSize = ( 40, 40 ), blockSize = ( 8, 8 ), blockStride = ( 4, 4 ),
				cellSize = ( 4, 4 ), nBins = 9, rectangleThreshold = 0.8, boundingBoxSize = 40 ):

		self.ballThreshold = ballThreshold
		self.goalThreshold = goalThreshold

		#	define hog descriptor instance
		self.hogDescriptor = cv2.HOGDescriptor( winSize, blockSize, blockStride, cellSize, nBins )

		#	define bounding box instance
		self.boundingBoxListObject = BoundingBoxList( winSize, rectangleThreshold, boundingBoxSize )

		#	flag to check predictor is predict or not yet
		self.isPredict = False

		#	flag to check that have any bounding box
		self.ableToExtraction = True
		
		#	initial best bounding box attribute
		self.bestBoundingBox = None

	def extractFeature( self, image, whiteContours, objectPointLocation = 'center' ):

		#	get bounding box from white contours first
		self.boundingBoxListObject.getBoundingBox( image, whiteContours, objectPointLocation = objectPointLocation )

		if self.boundingBoxListObject.getNumberCandidate() == 0:
			return False

		#	compute feature vector
		for boundingBox in self.boundingBoxListObject.boundingBoxList:

			#	compute!		
			hogFeature = self.hogDescriptor.compute( boundingBox.roiImage )

			#	HACK!!! : Get gray scale
			# imageGray = boundingBox.convertImageToGrayScale( )

			# hogGrayFeature = self.hogDescriptor.compute( imageGray )

			#	get feature vector
			boundingBox.featureVector = hogFeature.T
			# boundingBox.featureVectorGray = hogGrayFeature.T

		return True

	def extractFeature2( self, image, whiteContours, objectPointLocation = 'center' ):
		
		#	get bounding box from white contours first
		self.boundingBoxListObject.getBoundingBox( image, whiteContours, objectPointLocation = objectPointLocation )

		if self.boundingBoxListObject.getNumberCandidate() == 0:
			return False

		featureList = list()

		#	compute feature vector
		for boundingBox in self.boundingBoxListObject.boundingBoxList:

			hogFeature = self.hogDescriptor.compute( boundingBox.roiImage )

			boundingBox.featureVector = hogFeature.T

			featureList.append( hogFeature.T )

		return np.vstack( tuple( featureList ) )

	def predict( self ):

		#	loop every bounding list
		for boundingObject in self.boundingBoxListObject.boundingBoxList:

			#	get score in form probability
			boundingObject.footballProbabilityScore =  1.0
			boundingObject.goalProbabilityScore = 1.0

	def getBestRegion( self ):
		"""
			
			return None or tuple position
			
		"""

		#	initial best position parameter
		bestPosition = None
		confidence = 0.0
				
		#	flag to check that at least, one bounding box is football
		foundBall = False
		
		#	re-initial bounding box object
		self.bestBoundingBox = None

		# #	calculate
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
				if boundingBoxObj.footballProbabilityScore > self.ballThreshold :

					#	save it
					self.boundingBoxListObject.setPreviousBoundingBox( boundingBoxObj )
					
					#	get best bounding box
					self.bestBoundingBox = boundingBoxObj
					
					#	get position
					bestPosition = boundingBoxObj.object2DPosTuple
					confidence = boundingBoxObj.footballProbabilityScore
					
					#	found ball is true
					foundBall = True
					
					#	break the loop
					break
					
		else:

		# #	Sort score
		# self.boundingBoxListObject.sortProbScore()

			#	get best bounding box from sorted list by probability score
			self.bestBoundingBox = self.boundingBoxListObject.boundingBoxList[ 0 ]
			
			#	get position
			bestPosition = self.bestBoundingBox.object2DPosTuple
			confidence = self.bestBoundingBox.footballProbabilityScore
			
			#	set previous bounding box
			self.boundingBoxListObject.setPreviousBoundingBox( self.bestBoundingBox )
			
			foundBall = True
			
		#	clear bounding box list
		self.boundingBoxListObject.clearBoundingBoxList()

		return foundBall
		
	def getBestBoundingBox( self ):
		"""
			Must call after get best region
		"""
		
		return self.bestBoundingBox	

	def getGoal( self ):

		sortBoundingBoxGoalList = self.boundingBoxListObject.sortGoalScore()
		goalList = list()

		for goalObj in sortBoundingBoxGoalList:
			if goalObj.goalProbabilityScore > self.goalThreshold:
				confidence = goalObj.goalProbabilityScore
				goalList.append( goalObj )

		return goalList
			
		
	def printScore( self ):
		
		print "###################################"

		#	loop every bounding list
		for boundingObject in self.boundingBoxListObject.boundingBoxList:
			
			print "Candidate at {}".format( boundingObject.object2DPosTuple )

			print "ball : {}".format(  boundingObject.footballProbabilityScore )			
			print "goal : {}\n".format(  boundingObject.goalProbabilityScore )

		print "###################################"


class HOG_SVM( HOG_predictor ):

	def __init__( self, modelBallPathStr, modelGoalPathStr, ballThreshold = 0.5, goalThreshold = 0.5, winSize = ( 40, 40 ), blockSize = ( 8, 8 ), blockStride = ( 4, 4 ),
				cellSize = ( 4, 4 ), nBins = 9, rectangleThreshold = 0.8, boundingBoxSize = 10 ):

		super( HOG_SVM, self ).__init__( ballThreshold = ballThreshold, goalThreshold = goalThreshold,
										winSize = winSize, blockSize = blockSize, 
										blockStride = blockStride,	cellSize = cellSize, 
										nBins = nBins, rectangleThreshold = rectangleThreshold, 
										boundingBoxSize = boundingBoxSize )

		# #   get model for classifier
		self.footballModel = loadModel( modelBallPathStr )
		self.goalModel = loadModel( modelGoalPathStr )

		#HACK: Test svm from opencv
		svm = cv2.ml.SVM_create()
		self.svmBallModel = svm.load( '/home/neverholiday/work/ball_detector/src/model/cv_second_model_ball.yaml' )
		self.svmGoalModel = svm.load( '/home/neverholiday/work/ball_detector/src/model/cv_second_model_goal.yaml' )

	def predict( self ):

		#	loop every bounding list
		for boundingObject in self.boundingBoxListObject.boundingBoxList:

			#	get feature of bounding box
			hogFeature = boundingObject.featureVector

			#	get score in form probability
			boundingObject.footballProbabilityScore =  self.footballModel.predict_proba( hogFeature )[ 0, 1 ]
			boundingObject.goalProbabilityScore = self.goalModel.predict_proba( hogFeature )[ 0, 1 ]


	def predict2( self, sample ):

				#	loop every bounding list
		for boundingObject in self.boundingBoxListObject.boundingBoxList:

			#	get feature of bounding box
			hogFeature = boundingObject.featureVector

			#	get score in form probability
			boundingObject.footballProbabilityScore =  self.svmBallModel.predict( hogFeature, cv2.ml.STAT_MODEL_RAW_OUTPUT )[ 1 ]
			boundingObject.goalProbabilityScore = self.svmGoalModel.predict( hogFeature, cv2.ml.STAT_MODEL_RAW_OUTPUT )[ 1 ]


		# footballScore = self.svmModel.predict( sample )[ 1 ]
		# goalProbabilityScore = self.svmModel.predict( sample )[ 1 ]

		# print "	HOG_SVM::predict2 shape output : {}".format( footballScore.shape, goalProbabilityScore.shape )

		# return footballScore, goalProbabilityScore

class HOG_MLP( HOG_predictor ):

	def __init__( self, modelPath, ballThreshold = 0.5, goalThreshold = 0.5, winSize = ( 40, 40 ), blockSize = ( 8, 8 ), blockStride = ( 4, 4 ),
				cellSize = ( 4, 4 ), nBins = 9, rectangleThreshold = 0.8, boundingBoxSize = 10 ):

		super( HOG_MLP, self ).__init__( ballThreshold = ballThreshold, goalThreshold = goalThreshold, 
										winSize = winSize, blockSize = blockSize, 
										blockStride = blockStride,	cellSize = cellSize, 
										nBins = nBins, rectangleThreshold = rectangleThreshold, 
										boundingBoxSize = boundingBoxSize )

		self.model = loadModel( modelPath )

	def predict( self ):
		predictions = []
		#	loop every bounding list
		for boundingObject in self.boundingBoxListObject.boundingBoxList:

			prediction = self.model.predict_proba( boundingObject.featureVector )
			predictions.append( prediction )
			#	get score in form probability
			boundingObject.footballProbabilityScore = prediction[0, 0] 
			boundingObject.goalProbabilityScore = prediction[0, 1]

		print predictions

class HOG_CV2( HOG_predictor ):

	HIT_SIZE = 32

	def __init__( self, modelBallPathStr, modelGoalPathStr, ballThreshold = 0.5, goalThreshold = 0.5, winSize = ( 40, 40 ), blockSize = ( 8, 8 ), blockStride = ( 4, 4 ),
				cellSize = ( 4, 4 ), nBins = 9, rectangleThreshold = 0.8, boundingBoxSize = 10 ):

		super( HOG_CV2, self ).__init__( ballThreshold = ballThreshold, goalThreshold = goalThreshold,
										winSize = winSize, blockSize = blockSize, 
										blockStride = blockStride,	cellSize = cellSize, 
										nBins = nBins, rectangleThreshold = rectangleThreshold, 
										boundingBoxSize = boundingBoxSize )

		svm = cv2.ml.SVM_create()

		self.svm_ball = svm.load( modelBallPathStr )
		self.svm_goal = svm.load( modelGoalPathStr )

	def getHistogram( self, img, normalize = True ):

		hist = cv2.calcHist( [img], [0], None, [self.HIT_SIZE], [0,256] ).reshape( 1, -1 )
		if normalize:
			hist = hist / np.max( hist )
		return hist

	def extractFeature( self, image, whiteContours, objectPointLocation = 'center' ):

		# #	get bounding box from white contours first
		# self.boundingBoxListObject.getBoundingBox( image, whiteContours, objectPointLocation = objectPointLocation )

		# if self.boundingBoxListObject.getNumberCandidate() == 0:
		# 	return False

		# #	compute feature vector
		# for boundingBox in self.boundingBoxListObject.boundingBoxList:

		# 	#	compute!		
		# 	hogFeature = self.hogDescriptor.compute( boundingBox.roiImage )

		# 	#	HACK!!! : Get gray scale
		# 	# imageGray = boundingBox.convertImageToGrayScale( )

		# 	# hogGrayFeature = self.hogDescriptor.compute( imageGray )

		# 	#	get feature vector
		# 	boundingBox.featureVector = hogFeature.T
		# 	# boundingBox.featureVectorGray = hogGrayFeature.T

		# return True

		isExtract = super( HOG_CV2, self ).extractFeature( image, whiteContours, objectPointLocation = objectPointLocation )

		if not isExtract:
			return False

		for boundingBox in self.boundingBoxListObject.boundingBoxList:
			img = boundingBox.roiImage

			hist = self.getHistogram( img )

			boundingBox.featureVector = np.hstack((boundingBox.featureVector, hist))

		return True

	def predict( self ):

		## Get all bbox
		bboxList = []

		for boundingObj in self.boundingBoxListObject.boundingBoxList:
			score = self.svm_ball.predict( boundingObj.featureVector, -1, cv2.ml.STAT_MODEL_RAW_OUTPUT )[1]
			
			score = 1 / (1 + math.exp(-score[0,0]))
			boundingObj.footballProbabilityScore = 1 - score

			score = self.svm_goal.predict( boundingObj.featureVector, 0, cv2.ml.STAT_MODEL_RAW_OUTPUT )[1]
			score = 1 / (1 + math.exp(-score[0,0]))			
			boundingObj.goalProbabilityScore = 1 - score