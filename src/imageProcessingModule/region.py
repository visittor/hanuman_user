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

########################################################
#
#	CLASS DEFINITIONS
#

class BoundingBoxDataStruct( object ):

	def __init__( self, roiImage, topLeftPositionTuple, bottomRightPositionTuple, positionObject = 'center' ):

		#	get basic property
		self.roiImage = roiImage

		self.topLeftPositionTuple = topLeftPositionTuple
		self.bottomRightPositionTuple = bottomRightPositionTuple

		#	initial object position
		self.object2DPosTuple = self.calculateObjectPoint( positionObject )

		#	initial other attribrute
		self.footballProbabilityScore = 0.0
		self.goalProbabilityScore = 0.0
		self.pointPolygonScore = 0.0

		self.isFootball = False
		self.distanceFromPreviousPosition = 0.0
		self.featureVector = None

		#	Hack!!!
		self.featureVectorGray = None

	def calculateObjectPoint( self, positionObject ):
		
		width = self.bottomRightPositionTuple[ 0 ] - self.topLeftPositionTuple[ 0 ]
		height = self.bottomRightPositionTuple[ 1 ] - self.topLeftPositionTuple[ 1 ]

		if positionObject == 'center':
			
			x = self.topLeftPositionTuple[ 0 ] + width / 2
			y = self.topLeftPositionTuple[ 1 ] + height / 2
			pointsTuple = ( x, y )

		elif positionObject == 'top':
			
			x = self.topLeftPositionTuple[ 0 ] + width / 2
			y = self.topLeftPositionTuple[ 1 ]
			pointsTuple = ( x, y )

		elif positionObject == 'bottom':
			
			x = self.topLeftPositionTuple[ 0 ] + width / 2
			y = self.topLeftPositionTuple[ 1 ] + height
			pointsTuple = ( x, y )

		return pointsTuple

	def convertImageToGrayScale( self ):

		imgGray = cv2.cvtColor( self.roiImage, cv2.COLOR_BGR2GRAY )

		return imgGray


class BoundingBoxList( object ):

	def __init__( self, rectangleThreshold, boundingBoxSize ):
		
		#	get attribute filter
		self.rectangleThreshold = rectangleThreshold
		self.boundingBoxSize = boundingBoxSize

		#	list for store bounding box
		self.boundingBoxList = list()
		
		#	initial bounding data which the best of previous
		self.previousBoundingBox = None

	def getBoundingBox( self, image, whiteContours, imageSize = 40, objectPointLocation = 'center' ):

		#	get all bounding box
		allBoundingBoxList = map( cv2.boundingRect, whiteContours )

		#	filter by size and rectangle-like
		filteredBoundingBoxList = self.filterBoundingBox( allBoundingBoxList )
		
		for boundingBox in filteredBoundingBoxList:
			
			#	get roi image
			imageROI, topLeftPositionTuple, bottomRightPositionTuple = self.getImageROI( image, boundingBox )

			#	resize first
			imageROIResized = cv2.resize( imageROI, ( imageSize, imageSize ) )

			#	initial instance of data structure
			boundingBoxStruct = BoundingBoxDataStruct( imageROIResized, topLeftPositionTuple, bottomRightPositionTuple, objectPointLocation )

			#	append to the list
			self.boundingBoxList.append( boundingBoxStruct )

	def calculateDistanceFromPreviousBoundingBox( self ):
		
		#	if previous ball is None, ranking true from svm to top order and terminate this function
		if self.previousBoundingBox is None:

			self.boundingBoxList = sorted( self.boundingBoxList, key = lambda boundingBoxObj : boundingBoxObj.footballProbabilityScore, reverse=True )

			return False

		#	loop and put score to ball confidence
		for boundingBox in self.boundingBoxList:
			
			#	calculate distance by using eucledian distance
			#	calculate delta x and delta y
			deltaX = self.previousBoundingBox.object2DPosTuple[ 0 ] - boundingBox.object2DPosTuple[ 0 ]
			deltaY = self.previousBoundingBox.object2DPosTuple[ 1 ] - boundingBox.object2DPosTuple[ 1 ]

			#	calculate distance
			distance = np.sqrt( np.power( deltaX, 2 ) + np.power( deltaY, 2 ) )

			#	push the distance to object
			boundingBox.distanceFromPreviousPosition = distance
		
		return True
	
	def sortGoalScore( self ):

		sortBoundingBoxGoalList = sorted( self.boundingBoxList, key = lambda boundingBoxObj : boundingBoxObj.goalProbabilityScore, reverse=True )

		return sortBoundingBoxGoalList

	def getPreviousBoundingBox( self ):
		
		"""
			Get previous bounding box
		"""
		
		return self.previousBoundingBox
	
	def setPreviousBoundingBox( self, boundingBoxRect ):
		
		"""
			Set previous bounding box
		"""
		
		self.previousBoundingBox = boundingBoxRect

	def clearBoundingBoxList( self ):

		#	flush all element in list
		self.boundingBoxList = list()

	def getImageROI( self, image, boundingBox, expandPixelSize = 10 ):
		
		#	get image property : width, height
		imageWidth = image.shape[ 1 ]
		imageHeight = image.shape[ 0 ]

		#	get top-left position and bottom-right position
		x1, y1, x2, y2 = self.expandAreaBoundingBox( expandPixelSize, boundingBox, imageWidth, imageHeight )

		#	top-left and bottom right
		topLeftPositionTuple = ( x1, y1 )
		bottomRightPositionTuple = ( x2, y2 ) 

		#	image ROI
		roiImage = image[ y1 : y2, x1 : x2 ].copy()

		return roiImage, topLeftPositionTuple, bottomRightPositionTuple

	def expandAreaBoundingBox( self, pixelSize, boundingBox, imgWidth, imgHeight ):
	
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

	def filterBoundingBox( self, boundingBoxList ):

		#	filter by size and rectangle-like
		filterSizeFunction = lambda boundingBoxTuple : boundingBoxTuple[ 2 ] >= self.boundingBoxSize and boundingBoxTuple[ 3 ] >= self.boundingBoxSize
		filterNonRectFunction = lambda boundingBoxTuple : abs( boundingBoxTuple[ 2 ] - boundingBoxTuple[ 3 ] ) <= self.rectangleThreshold

		boundingBoxFilteredSizeList = filter( filterSizeFunction, boundingBoxList )
		boundingBoxFilteredNonRectList = filter( filterNonRectFunction, boundingBoxFilteredSizeList )

		return boundingBoxFilteredNonRectList

	def getNumberCandidate( self ):

		return len( self.boundingBoxList )



