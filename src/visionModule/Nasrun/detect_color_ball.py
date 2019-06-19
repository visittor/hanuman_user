#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun (NeverHoliday) Hayeeyama
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

from visionManager.visionModule import VisionModule, KinematicModule
from newbie_hanuman.msg import visionMsg, postDictMsg

from geometry_msgs.msg import Point32 

from forwardkinematic import getMatrixForForwardKinematic, loadDimensionFromConfig

from scanLine2 import findBoundary, findLinearEqOfFieldBoundary, findNewLineFromRansac

from imageProcessingModule.hog_svm import HOG_SVM
from imageProcessingModule import findGoal

import rospy

import numpy as np
import cv2

import configobj

import time

from scipy import spatial
from kinematic import Kinematic

########################################################
#
#	GLOBALS
#

#ModelPath = os.path.join( os.getenv( 'ROS_WS' ), 'src/hanuman_user/config/model/real_model_with_prob.pkl' )
ModelPath = os.path.join( os.getenv( 'ROS_WS' ), 'src/hanuman_user/config/model/model_gray.pkl' )

MagentaColorID = 7
OrangeColorID = 1


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

########################################################
#
#	CLASS DEFINITIONS
#

class ImageProcessing( VisionModule ):
	'''	ImageProcessingClass
	
	Do autosegment
	
	'''
	
	def __init__( self ):
		
		super( ImageProcessing, self ).__init__()
		
		#	define type
		self.objectsMsgType = visionMsg
		
		#	contour visualization
		self.contourFieldVis = None
		
	def calculateError( self, imageWidth, imageHeight, centerX, centerY ):

		errorX = ( centerX - imageWidth / 2. ) / ( imageWidth / 2. )
		errorY = ( centerY - imageHeight / 2. ) / ( imageHeight / 2. )

		return errorX, errorY
	
				
	def ImageProcessingFunction( self, img, header ):
		'''	ImageProcessingFunction
			USE WATERSHED BEFORE.
		'''
		
		#	get image property
		imageHeight = img.shape[ 0 ]
		imageWidth = img.shape[ 1 ]
		
		#	get gray scale image
		imageGray = img[ :, :, 0 ].copy()
		
		#	get marker (on channel 1)
		marker = img[ :, :, 1 ]  
		
		#	get field boundary
		fieldContour, fieldMask = findBoundary( marker, 2, flip = False )
		self.contourFieldVis = fieldContour
		 
		#
		#	get white object and predict ball
		#
		# whiteObject = np.zeros( marker.shape, dtype = np.uint8 )
		# whiteObject[ marker == 5 ] = 1
		
		# #	get white object only the field
		# whiteObjectInField = whiteObject * fieldMask
		# whiteObjectInField *= 255

		#	Get magenta object
		ballMagentaObject = np.zeros( marker.shape, dtype = np.uint8 )
		ballMagentaObject[ marker == MagentaColorID ] = 1
		ballMagentaObjectInField = fieldMask * ballMagentaObject
		ballMagentaObjectInField *= 255

		#	Get orange color object
		ballOrangeObject = np.zeros( marker.shape, dtype = np.uint8 )
		ballOrangeObject[ marker == OrangeColorID ] = 1
		ballOrangeObjectInField = fieldMask * ballOrangeObject
		ballOrangeObjectInField *= 255

		#	Get contour of orange and magenta
		ballMagentaObjectContour = cv2.findContours( ballMagentaObjectInField, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[1]
		ballOrangeObjectContour = cv2.findContours( ballOrangeObjectInField, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[1]
		
		objectNameList = list()
		pos2DList = list()
		objectErrorList = list()
		objectConfidenceList = list()

		if len( ballMagentaObjectContour ) != 0:
			x,y,w,h = cv2.boundingRect( ballMagentaObjectContour[ 0 ] )
			objectNameList.append( 'ball_magenta' )
			pos2DList.append( Point32( x = x + (w/2), y = y + (h/2), z = 0 ) )
			errorX, errorY = self.calculateError( imageWidth, imageHeight, x + (w/2), y + (h/2) )
			objectErrorList.append( Point32( x = errorX, y = errorY, z = 0.0 ) )
			objectConfidenceList.append( 1.0 )
		
		if len( ballOrangeObjectContour ) != 0:
			x, y, w, h = cv2.boundingRect( ballOrangeObjectContour[ 0 ] )
			objectNameList.append( 'ball_orange' )
			pos2DList.append( Point32( x = x + (w/2), y = y + (h/2), z = 0 ) )
			errorX, errorY = self.calculateError( imageWidth, imageHeight, x + (w/2), y + (h/2) )
		 	objectErrorList.append( Point32( x = errorX, y = errorY, z = 0.0 ) )
			objectConfidenceList.append( 1.0 )

		msg = self.createVisionMsg( objectNameList, pos2DList, objectErrorList, objectConfidenceList, imageWidth, imageHeight )
		
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

		super( ImageProcessing, self ).visualizeFunction( img, msg )

		if 'ball_magenta' in msg.object_name:
			idxMagenta = msg.object_name.index( 'ball_magenta' )
			x = msg.pos2D[ idxMagenta ].x
			y = msg.pos2D[ idxMagenta ].y
			cv2.circle( img, ( x, y ), 5, ( 255, 255, 255 ), -1 )
			cv2.putText( img, "ball_magenta", ( x, y ), cv2.FONT_HERSHEY_COMPLEX, 2, ( 255, 0, 0 ), 2 )
		
		if 'ball_orange' in msg.object_name:
			idxOrange = msg.object_name.index( 'ball_orange' )
			x = msg.pos2D[ idxOrange ].x
			y = msg.pos2D[ idxOrange ].y
			cv2.circle( img, ( x, y ), 5, ( 255, 255, 255 ), -1 )
			cv2.putText( img, "ball_orange", ( x, y ), cv2.FONT_HERSHEY_COMPLEX, 2, ( 255, 0, 0 ), 2 )		

vision_module = ImageProcessing()
kinematic_module = Kinematic()
