#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os

import pickle

########################################################
#
#	LOCAL IMPORTS
#

import cv2
import numpy as np

import rospy

from scanLine2 import findBoundary, findNewLineFromRansac

from imageProcessingModule import findGoal

from visionManager.visionModule import VisionModule

from newbie_hanuman.msg import visionMsg, postDictMsg
from geometry_msgs.msg import Point32 

from imageProcessingModule.hog import HogProvider

########################################################
#
#	GLOBALS
#

ModelPath = os.path.join( os.getenv( 'ROS_WS' ), 'src/hanuman_user/config/model/model_goal_gray.pkl' )

FieldGreenColorID = 2
WhiteColotID = 5

DefaultKernelShapeTuple = ( 20, 5 )
DefaultThreshold = 0,2

########################################################
#
#	EXCEPTION DEFINITIONS
#

########################################################
#
#	HELPER FUNCTIONS
#

def getJsPosFromName(Js, name):
	'''
	Get Joint state of pantilt.
	argument:
		Js 		:	Joint state message
		name 	:	(str) Either 'pan' or 'tilt'
	return:
		Position of that joint
	'''
	indexJs = Js.name.index( name )
	return Js.position[ indexJs ]

def loadPickle( path ):
	''' loadPickle function
	'''
	with open( path, 'r' ) as f:
		obj = pickle.load( f )

	return obj

def expandBoundingBox( topLeft, bottomRight, imageWidth, imageHeight, expandSize = 10 ):
	'''	expandBoundingBox function
	'''
	
	topLeft_X = topLeft[0]
	topLeft_Y = topLeft[1]

	bottomRight_X = bottomRight[0]
	bottomRight_Y = bottomRight[1]

	#	calculate expand fourpoints
	newTopLeft_X = max( 0, topLeft_X - expandSize )
	newTopLeft_Y = max( 0, topLeft_Y - expandSize )
	newBottomRight_X = min( imageWidth, bottomRight_X + expandSize )
	newBottomRight_Y = min( imageHeight, bottomRight_Y + expandSize )

	return newTopLeft_X, newTopLeft_Y, newBottomRight_X, newBottomRight_Y

def generateFourpoints( topLeft, width, height ):
	'''	createFourPoints function
	'''

	#		topLeft (origin) --------- topRight
	#				|					   |
	#				|					   |
	#				|					   |
	#	  	    bottomLeft	 ---------  bottomRight (final position) 

	#	NOTE: Arrange by bottomLeft -> topLeft -> topRight -> bottomRight

	#	topRight x0+w, y0
	topRight = ( topLeft[0] + width, topLeft[1] )

	#	bottomLeft x0, y0+h
	bottomLeft = ( topLeft[0], topLeft[1] + height )

	#	bottomRight x0+w,y0+h
	bottomRight = ( topLeft[0]+width, topLeft[1]+height )

	#	Pack four point list
	fourPointList = [ bottomLeft, topLeft, topRight, bottomRight ]

	return fourPointList
	

########################################################
#
#	CLASS DEFINITIONS
#

class ImageProcessing( VisionModule ):
	''' ImageProcessing Class
	'''

	def __init__( self ):

		#   Initial attribute of parent class
		super( ImageProcessing, self ).__init__()

		#   define type
		self.objectsMsgType = visionMsg

		#   load model
		self.model = loadPickle( ModelPath )

		#	create hog instance
		self.hog = HogProvider()

		#   initial list to store bounding box of white object
		self.boundingBoxList = list()

		#
		#	REMOVE LATER : FOR DEBUG
		#
		self.pointGoalList = list()
		self.pointVerifyByModel = None

	def ImageProcessingFunction( self, img, header ):
		''' ImageProcessingFunction function
		'''

		#   Re-initialize
		self.boundingBoxList = list()

		#	ROI image list
		imageROIList = list()

		#	Position list -> convert to numpy after that
		positionList = list()

		#   Get property of image
		imageHeight = img.shape[ 0 ]
		imageWidth = img.shape[ 1 ]

		#   Get grayscale image
		grayImage = img[ :, :, 0 ].copy()

		#   Get marker
		marker = img[ :, :, 1 ].copy()

		#   Find field boundary
		fieldContour, fieldMask = findBoundary( marker, FieldGreenColorID )

		#   Get new contour from ransac
		ransacContours = findNewLineFromRansac( fieldContour, imageWidth, imageHeight )

		#   Create mask from new contour
		newFieldMask = np.zeros( marker.shape, dtype=np.uint8 )
		cv2.drawContours( newFieldMask, [ ransacContours ], 0, 1, -1 )

		#   Get white object mask
		whiteObjectMask = np.zeros( marker.shape, dtype=np.uint8 )
		whiteObjectMask[ marker == WhiteColotID ] = 1

		#   Eliminate outer field
		whiteObjectInFieldMask = whiteObjectMask * newFieldMask * 255

		#   TRY!!!
		#   Opening for eliminate white field
		kernel = np.ones( DefaultKernelShapeTuple, dtype=np.uint8 )
		whiteObjectInFieldMask = cv2.morphologyEx( whiteObjectInFieldMask, cv2.MORPH_OPEN, kernel )

		#   NOTE: I should find ball first
		
		#   Find contour of white object in field
		whiteObjectContours = cv2.findContours( whiteObjectInFieldMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]
		if len( whiteObjectContours ) != 0:
			for cnt in whiteObjectContours:
				
				#	Get bounding box (x,y,width,height)
				x, y, w, h = cv2.boundingRect( cnt )

				if w < 10 or h < 10:
					continue

				# if w > 100 or h > 100:
				# 	continue

				#	append to position list
				centerX = x + w / 2
				centerY = y + h / 2
				positionList.append( ( centerX, centerY ) )

				#	Get top and left point
				topLeft = ( x, y )
				bottomRight = ( x + w, y + h )

				#	Expand area
				newTopLeft_X, newTopLeft_Y, newBottomRight_X, newBottomRight_Y = expandBoundingBox( topLeft, bottomRight, 
																									imageWidth, imageHeight )
				
				#	Get new width and height
				newWidth = newBottomRight_X - newTopLeft_X
				newHeight = newBottomRight_Y - newTopLeft_Y

				#	Calculate new rectangle contours
				fourPointList = generateFourpoints( ( newTopLeft_X, newTopLeft_Y ), newWidth, newHeight )
				
				#	Change to form of contours
				fourPointContours = np.array( fourPointList ).reshape( -1, 1, 2 )

				#	Append new fourpoints to the list
				self.boundingBoxList.append( fourPointContours )

				#	Crop image from gray scale image
				imgROI = grayImage[ newTopLeft_Y : newBottomRight_Y, newTopLeft_X : newBottomRight_X ].copy()

				#	Resize to 40, 40
				imgROI = cv2.resize( imgROI, ( 40, 40 ) )

				#	Append to image list
				imageROIList.append( imgROI )

			#	Change position to numpy array
			positonMatrix = np.array( positionList )

			#	Get feature matrix dim = (nSample, nFeatureVectors)
			featureMatrix = self.hog.extract( imageROIList )

			#	Get probability score of each sample
			probabilityScoreVector = self.model.predict_proba( featureMatrix )[ :, 1 ]

			#	Find score higher than threshold
			
			# print filteredPosition

		#	old approach !!!
		pointGoalList = findGoal( ransacContours, marker )

		#	Create other score vector
		pointPolygonScoreVector = np.zeros( probabilityScoreVector.shape, dtype=float )		
		
		if pointGoalList is not None:
			self.pointGoalList = pointGoalList

			#	Loop point first
			for p in pointGoalList:
				
				#		Loop each bounding rectangle
				for idx, cnt in enumerate( self.boundingBoxList ):

					#	Get width and height of rectangle contours
					width = cnt[ 2, 0, 0 ] - cnt[ 1, 0, 0 ]
					height = cnt[ 0, 0, 1 ] - cnt[ 1, 0, 1 ]
	
					#	If there have any point that inside rectangle
					score = cv2.pointPolygonTest( cnt, p, True )
					if score > 0:
					
						#	Calculate normalize score (0-1) and plus score to that bounding box
						score /= ( min( width, height ) / 2.0 )
						pointPolygonScoreVector[ idx ] += score

		print "Score from model of each candidate : {}".format( probabilityScoreVector )
		print "Score from check point : {}".format( pointPolygonScoreVector )

		#	Initial msg
		msg = visionMsg()
		# msg.header.stamp = rospy.Time.now()
		# msg.object_name = list()
		# msg.pos2D = list()
		# msg.object_error = list() # Later...
		# msg.object_confidence = list() # Later...
		# msg.imgH = imageHeight
		# msg.imgW = imageWidth

		# #	Send goal first
		# for i, point in enumerate( filteredPosition ):

		# 	msg.object_name.append( 'goal_{}'.format( i+1 ) )
		# 	msg.pos2D.append( Point32( point[0],point[1],0 ) )

		#   Return msg    
		# msg.header.stamp = rospy.Time.now()
		# msg.object_name = [ 'ball', ]
		# msg.pos2D = [ Point32() ]
		# msg.imgH = imageHeight
		# msg.imgW = imageWidth
		# msg.object_error = [ Point32() ]
		# msg.object_confidence = [ 0.0 ]

		return msg

	def visualizeFunction(self, img, msg):
		''' visualizeFunction function
		'''
		for colorDict in self.colorConfig.colorDefList:
			img[ img[:,:,1] == colorDict.ID ] = eval(colorDict.RenderColor_RGB)

		for cnt in self.boundingBoxList:

			cv2.drawContours( img, [ cnt ], 0, ( 255, 0, 0 ), 2, cv2.LINE_AA )

		for p in self.pointGoalList:

			cv2.circle( img, p, 5, ( 0, 0, 255 ), -1 )