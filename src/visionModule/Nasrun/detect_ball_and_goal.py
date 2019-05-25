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

import cv2
import numpy as np

import rospy
import configobj

import time

########################################################
#
#	LOCAL IMPORTS
#

from imageProcessingModule.hog_svm import HOG_SVM

from colorSegmentation import colorSegmentation, createColorDefFromDict

from scanLine2 import findBoundary, findNewLineFromRansac

from configobj import ConfigObj

from visionManager.visionModule import VisionModule, KinematicModule
from newbie_hanuman.msg import visionMsg, postDictMsg

from geometry_msgs.msg import Point32 

########################################################
#
#	GLOBALS
#

FootballModelPath =  os.path.join( os.getenv( 'ROS_WS' ), "src/hanuman_user/config/model/real_model_with_prob.pkl" )
GoalModelPath = os.path.join( os.getenv( 'ROS_WS' ), "src/hanuman_user/config/model/model_goal_gray.pkl" ) 

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

class ImageProcessing( VisionModule ):
	'''
	NOTE: not do autosegment, refactor later 
	'''

	def __init__( self ):

		super( ImageProcessing, self ).__init__()

				#	define type
		self.objectsMsgType = visionMsg
		
		#	get color config file path from rosparam
		robotConfigPathStr = rospy.get_param( '/robot_config', None )

		if robotConfigPathStr is None:
			raise TypeError( 'Required robot config.' )
		
		#	get model object
		#	positive threshold is 0.70
		self.predictor = HOG_SVM( FootballModelPath, GoalModelPath, 0.80, rectangleThreshold=30 )

		#	get color definition from color config ( get only values )
		colorConfigList = configobj.ConfigObj( robotConfigPathStr )[ "ColorDefinitions" ]
		colorConfigList = colorConfigList.values()

		#	create color definition for using segmentation 
		self.colorDefList = createColorDefFromDict( colorConfigList )

	def calculateError( self, imageWidth, imageHeight, centerX, centerY ):

		errorX = ( centerX - imageWidth / 2. ) / ( imageWidth / 2. )
		errorY = ( centerY - imageHeight / 2. ) / ( imageHeight / 2. )

		return errorX, errorY

	def ImageProcessingFunction( self, img, header ):

		startTime = time.time()

		objNameList = list()
		pos2DList = list()
		errorList = list()
		confidenceList = list()

		imageWidth = img.shape[ 1 ]
		imageHeight = img.shape[ 0 ]

		blurImage = cv2.GaussianBlur( img, (5,5), 0 )
		hsvImage = cv2.cvtColor( blurImage, cv2.COLOR_BGR2HSV )

		marker = colorSegmentation( blurImage, self.colorDefList )
		marker = cv2.watershed( hsvImage, marker )

		fieldContour, fieldMask = findBoundary( marker, 2 )
		ransacContours, coeff = findNewLineFromRansac( fieldContour, imageWidth, imageHeight )

		if len( coeff ) > 1:
			#	find y intersect
			xIntersect = coeff[ 0 ][ 3 ]
			m = coeff[ 0 ][ 0 ]
			c = coeff[ 0 ][ 1 ]

			yIntersect = ( m * xIntersect ) + c
			
			
			intersectPoint = Point32( x = xIntersect, y = yIntersect, z = 0.0 )
			errorX, errorY = self.calculateError( imageWidth, imageHeight, xIntersect, yIntersect )
			
			errorIntersectPoint = Point32( x = errorX, y = errorY, z = 0.0 )
			intersectPointConfidence = 1.0
			
			objNameList.append( 'field_corner' )
			pos2DList.append( intersectPoint )
			errorList.append( errorIntersectPoint )
			confidenceList.append( intersectPointConfidence )

		#   Create mask from new contour
		newFieldMask = np.zeros( marker.shape, dtype=np.uint8 )
		cv2.drawContours( newFieldMask, [ ransacContours ], 0, 1, -1 )

		whiteObjectMask = np.zeros( marker.shape, dtype=np.uint8 )
		whiteObjectMask[ marker == 5 ] = 1

		whiteObjectInFieldMask = whiteObjectMask * newFieldMask * 255
		whiteObjectContours = cv2.findContours( whiteObjectInFieldMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

		canExtract = self.predictor.extractFeature( img, whiteObjectContours, objectPointLocation="bottom" )

		if canExtract:

			self.predictor.predict()

			goalList = self.predictor.getGoal()
			bestPosition = tuple(self.predictor.getBestRegion())

			if len( bestPosition ) != 0:

				objNameList.append( 'ball' )

				#	get bounding box object not only position
				bestBounding = self.predictor.getBestBoundingBox()
				#	get another point of object
				centerX, centerY = bestBounding.calculateObjectPoint( 'center' )

				pos2DList.append( Point32( bestPosition[ 0 ][ 0 ],
										   bestPosition[ 0 ][ 1 ],
										   0.0 ) )
				#	calculate error
				errorX, errorY = self.calculateError( imageWidth, imageHeight, centerX, centerY )
				errorList.append( Point32( errorX, errorY, 0.0 ) )

				confidenceList.append( bestPosition[ 1 ] )

			for goal in goalList:

				objNameList.append( 'goal' )
				pos2DList.append( Point32( goal[ 0 ][ 0 ],
										   goal[ 0 ][ 1 ],
										   0.0 ) )
				
				errorX, errorY = self.calculateError( imageWidth, imageHeight, goal[ 0 ][ 0 ], goal[ 0 ][ 1 ] )
				errorList.append( Point32( errorX, errorY, 0.0 ) )

				confidenceList.append( goal[ 1 ] )

		msg = self.createVisionMsg( objNameList, pos2DList, errorList, confidenceList, imageWidth, imageHeight )
 
		rospy.logdebug( "Time usage : {}".format( time.time() - startTime ) )

		return msg


	def createVisionMsg( self, objectNameList, pos2DList, objectErrorList, objectConfidenceList, imgWidth, imgHeight ):
		'''	createVisionMsg function
		'''
		msg = visionMsg()
		msg.header.stamp = rospy.Time.now()
		msg.object_name = objectNameList
		msg.pos2D = pos2DList
		msg.object_error = objectErrorList
		msg.object_confidence = objectConfidenceList
		msg.imgH = imgHeight
		msg.imgW = imgWidth

		return msg

	def visualizeFunction(self, img, msg):
		"""For visualization by using cranial nerve monitor"""

		
		if 'ball' in msg.object_name:
			ballIdx = msg.object_name.index( 'ball' )
			x = msg.pos2D[ ballIdx ].x
			y = msg.pos2D[ ballIdx ].y
			cv2.circle( img, ( x, y ), 5, ( 255, 0, 0 ), -1 )
			cv2.putText( img, "ball", ( x, y ), cv2.FONT_HERSHEY_COMPLEX, 2, ( 255, 0, 0 ), 2 )

		for i in msg.object_name:
			if i == 'goal':
				idx = msg.object_name.index( 'goal' )
				if msg.object_confidence[ idx ] > 0.5:
					x = msg.pos2D[ idx ].x
					y = msg.pos2D[ idx ].y
					cv2.circle( img, ( x, y ), 5, ( 0, 0, 255 ), -1 )
					cv2.putText( img, "goal", ( x, y ), cv2.FONT_HERSHEY_COMPLEX, 2, ( 0, 0, 255 ), 2 )

		if 'field_corner' in msg.object_name:
			cornerIdx = msg.object_name.index( 'field_corner' )
			x = int(msg.pos2D[ cornerIdx ].x)
			y = int(msg.pos2D[ cornerIdx ].y)
			cv2.circle( img, ( x, y ), 5, ( 255, 0, 0 ), -1 )
			cv2.putText( img, "corner", ( x, y ), cv2.FONT_HERSHEY_COMPLEX, 2, ( 255, 0, 0 ), 2 )