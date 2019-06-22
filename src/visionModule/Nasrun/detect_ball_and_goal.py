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

from imageProcessingModule.hog_svm import HOG_SVM, HOG_MLP

from colorSegmentation import colorSegmentation, createColorDefFromDict

from scanLine2 import findBoundary, findNewLineFromRansac, findLinearEqOfFieldBoundary
from scanLine2 import findChangeOfColor

from configobj import ConfigObj

from visionManager.visionModule import VisionModule, KinematicModule
from utility.HanumanForwardKinematic import loadDimensionFromConfig, getMatrixForForwardKinematic
from utility.transformationModule import Project3Dto2D, Project2Dto3D, getInverseHomoMat

from newbie_hanuman.msg import visionMsg, postDictMsg

from geometry_msgs.msg import Point32 

import math
import time

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
		self.subscribePantiltTopic = True

		
		#	get color config file path from rosparam
		robotConfigPathStr = rospy.get_param( '/robot_config', None )

		if robotConfigPathStr is None:
			raise TypeError( 'Required robot config.' )
		
		#	get model object
		#	positive threshold is 0.70
		self.predictor = HOG_SVM( FootballModelPath, GoalModelPath, 0.80, rectangleThreshold=0.5, boundingBoxSize=10 )
		# self.predictor = HOG_MLP( '/home/visittor/Downloads/Dataset/Dataset/model/train_SVM_model.pk1', 
		# 						0.80, rectangleThreshold=30 )

		#	get color definition from color config ( get only values )
		loadDimensionFromConfig( robotConfigPathStr )
		self.config = configobj.ConfigObj( robotConfigPathStr )
		fx = float( self.config['CameraParameters']['fx'] )
		fy = float( self.config['CameraParameters']['fy'] )
		cx = float( self.config['CameraParameters']['cx'] )
		cy = float( self.config['CameraParameters']['cy'] )
		imgW = float( self.config['CameraParameters']['imgW'] )
		imgH = float( self.config['CameraParameters']['imgH'] )
		self.scale = float( self.config['CameraParameters']['scale'] )

		self.cameraMatrix = np.array( [ [fx, 0, cx], [0, fy, cy], [0, 0, 1] ] )

		self.tiltForLookAtFoot = float(self.config['PanTiltPlanner'].get( 'TiltAngleForLookAtFoot', 60 ))
		self.tiltForLookAtFoot = math.radians( self.tiltForLookAtFoot )

	def _setColorConfig( self, colorConfig ):

		super( ImageProcessing, self )._setColorConfig( colorConfig )

		self.greenID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'green' ][ 0 ]
		self.whiteID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'white' ][ 0 ]

	def calculateError( self, imgW, imgH, centerX, centerY ):

		errorX = ( centerX - imgW / 2. ) / ( imgW / 2. )
		errorY = ( centerY - imgH / 2. ) / ( imgH / 2. )

		return errorX, errorY

	def getWhiteObjectContour( self, marker, fieldContour ):

		#   Create mask from new contour
		fieldMask = np.zeros( marker.shape, dtype=np.uint8 )
		cv2.drawContours( fieldMask, [ fieldContour ], 0, 1, -1 )

		whiteObjectMask = np.zeros( marker.shape, dtype=np.uint8 )
		whiteObjectMask[ marker == self.whiteID ] = 1

		whiteObjectInFieldMask = cv2.bitwise_and(whiteObjectMask, fieldMask) * 255
		kernel = np.ones( (5,5), dtype=np.uint8 )
		whiteObjectInFieldMask = cv2.morphologyEx( whiteObjectInFieldMask, cv2.MORPH_OPEN, kernel )
		whiteObjectInFieldMask = cv2.morphologyEx( whiteObjectInFieldMask, cv2.MORPH_CLOSE, kernel )

		whiteObjectContours = cv2.findContours( whiteObjectInFieldMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

		## Delete line segment from mask.
		kernel = np.ones( (25,1), dtype=np.uint8 )
		whiteObject_noline = cv2.morphologyEx( whiteObjectInFieldMask, cv2.MORPH_OPEN, kernel )
		whiteObjectContours_noline = cv2.findContours( whiteObject_noline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

		whiteObjectContours.extend( whiteObjectContours_noline )

		# ## Delete line segment from mask.
		# kernel = np.ones( (50,1), dtype=np.uint8 )
		# whiteObject_noline = cv2.morphologyEx( whiteObjectInFieldMask, cv2.MORPH_OPEN, kernel )
		# whiteObjectContours_noline = cv2.findContours( whiteObject_noline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

		# whiteObjectContours.extend( whiteObjectContours_noline )
		
		return whiteObjectContours

	def findKickCandidate( self, whiteMask ):
		imgH, imgW = whiteMask.shape[:2]

		yOffset = 80

		centerL = imgW / 4, (imgH - yOffset) / 2
		centerR = (3*imgW )/ 4, (imgH - yOffset) / 2

		w = imgW / 4
		h = ( imgH - yOffset ) / 2

		LSide = whiteMask[centerL[1]-h:centerL[1]+h, centerL[0]-w:centerL[0]+w]
		RSide = whiteMask[centerR[1]-h:centerR[1]+h, centerR[0]-w:centerR[0]+w]

		sumL = np.sum( LSide )
		sumR = np.sum( RSide )

		return centerL if sumL > sumR else centerR

	def addObject( self, x, y, name, confidence, imgSize, nameList, pos2DList, 
					confidenceList, errorList, error = None ):

		nameList.append( name )
		pos2DList.append( Point32( x = int( x/self.scale ), y = int( y/self.scale ), z = 0 ) )
		confidenceList.append( confidence )

		if error is not None:
			errX, errY = self.calculateError( imgSize[0], imgSize[1], error[0], error[1] )
		else:
			errX, errY = self.calculateError( imgSize[0], imgSize[1], x, y )

		errorList.append( Point32( x = errX, y = errY, z = 0.0 ) )

	def ImageProcessingFunction( self, img, header, pan_tilt = None ):

		startTime = time.time()

		objNameList = list()
		pos2DList = list()
		errorList = list()
		confidenceList = list()

		imgH, imgW = img.shape[ :2 ]

		marker = img[:, :, 1]
		gray = img[:, :, 0]

		t0 = time.time()
		fieldContour, fieldMask = findBoundary( marker, self.greenID )
		coeff = findLinearEqOfFieldBoundary( fieldContour )

		ransacContours =  [ [0, imgH] ]
		for m, c, x0, xf in coeff:
			ransacContours.append( [ x0, m*x0 + c ] )
			ransacContours.append( [ xf, m*xf + c ] )
		ransacContours.append( [imgW, imgH] )
		ransacContours = np.vstack( ransacContours ).astype(int).reshape(-1,1,2)

		t1 = time.time()

		pointClound = findChangeOfColor( marker, self.whiteID, self.greenID, 
										mask = fieldMask, step = 20 )
		
		t2 = time.time()

		for scanline in pointClound:

			for x,y in scanline:
				self.addObject( x, y, 'point', 1.0, (imgH,imgW),
							objNameList, pos2DList, confidenceList, errorList )

		for p in fieldContour[1:-1:20,0,:]:
			if p[1] < 10:
				continue

			self.addObject( p[0], p[1], 'field', 1.0, (imgH, imgW),
							objNameList, pos2DList, confidenceList, errorList )

		t3 = time.time()

		if len( coeff ) > 1:
			#	find y intersect
			xIntersect = coeff[ 0 ][ 3 ]
			m = coeff[ 0 ][ 0 ]
			c = coeff[ 0 ][ 1 ]

			yIntersect = ( m * xIntersect ) + c

			self.addObject( xIntersect, yIntersect, 'field_corner', 1.0, (imgW,imgH),
							objNameList, pos2DList, confidenceList, errorList )

		self.whiteObjectContours = self.getWhiteObjectContour( marker, ransacContours )

		t4 = time.time()

		self.kickCandidate = self.findKickCandidate( marker == 5 )

		self.addObject( self.kickCandidate[0], self.kickCandidate[1], 'kicking_side',
						1.0, (imgW,imgH), objNameList, pos2DList, confidenceList,
						errorList )

		t5 = time.time()

		canExtract = self.predictor.extractFeature( gray, self.whiteObjectContours, objectPointLocation="bottom" )

		numCandidate = self.predictor.boundingBoxListObject.getNumberCandidate()

		t6 = time.time()

		if canExtract:

			predictTimeStart = time.time()
			self.predictor.predict()
			predictTimeEnd = time.time()
			
			goalList = self.predictor.getGoal()
			foundBall = self.predictor.getBestRegion()

			if foundBall:

				#	get bounding box object not only position
				bestBounding = self.predictor.getBestBoundingBox()
				#	get another point of object
				botX, botY = bestBounding.bottom
				centerX, centerY = bestBounding.center

				self.addObject( botX, botY, 'ball', bestBounding.footballProbabilityScore,
								(imgW,imgH), objNameList, pos2DList, confidenceList,
								errorList, error=(centerX,centerY) )

			for goalObj in goalList:

				botX, botY = goalObj.bottom

				self.addObject( botX, botY, 'goal', goalObj.goalProbabilityScore,
								(imgW,imgH), objNameList, pos2DList, confidenceList,
								errorList )

		t7 = time.time()

		msg = self.createVisionMsg( objNameList, pos2DList, errorList, confidenceList, imgW, imgH )

		t8 = time.time()
 
		rospy.logdebug( "Time usage : {}".format( time.time() - startTime ) )
		rospy.logdebug( "	Time ransac : {}".format( t1 - t0 ) )
		rospy.logdebug( "	Time point clound : {}".format( t2 - t1 ) )
		rospy.logdebug( "	Time append pc : {}".format( t3 - t2 ) )
		rospy.logdebug( "	Time white cnt : {}".format( t4- t3 ) )
		rospy.logdebug( "	Time kick cand : {}".format( t5 - t4 ) )
		rospy.logdebug( "	Time ext feat : {}".format( t6 - t5 ) )
		rospy.logdebug( "	Number of candidate : {}".format( numCandidate ) )
		rospy.logdebug( "	Time predict : {}".format( t7 - t6 ) )
		rospy.logdebug( "	Time predict only : {}".format( predictTimeEnd - predictTimeStart ) )
		rospy.logdebug( "	Time create msg : {}".format( t8 - t7 ) )

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
		super( ImageProcessing, self ).visualizeFunction( img, msg )

		cv2.drawContours( img, self.whiteObjectContours, -1, (0,0,255), 1 )

		cv2.circle( img, self.kickCandidate, 5, (0,0,255), -1 )

		if 'ball' in msg.object_name:
			ballIdx = msg.object_name.index( 'ball' )
			x = int( msg.pos2D[ ballIdx ].x / 2 )
			y = int( msg.pos2D[ ballIdx ].y / 2 )
			cv2.circle( img, ( x, y ), 5, ( 255, 0, 0 ), -1 )
			cv2.putText( img, "ball", ( x, y ), cv2.FONT_HERSHEY_COMPLEX, 2, ( 255, 0, 0 ), 2 )

		for i in msg.object_name:
			if i == 'goal':
				idx = msg.object_name.index( 'goal' )
				if msg.object_confidence[ idx ] > 0.5:
					x = int(msg.pos2D[ idx ].x/2)
					y = int(msg.pos2D[ idx ].y/2)
					cv2.circle( img, ( x, y ), 5, ( 0, 0, 255 ), -1 )
					cv2.putText( img, "goal", ( x, y ), cv2.FONT_HERSHEY_COMPLEX, 2, ( 0, 0, 255 ), 2 )

		if 'field_corner' in msg.object_name:
			cornerIdx = msg.object_name.index( 'field_corner' )
			x = int(msg.pos2D[ cornerIdx ].x/2)
			y = int(msg.pos2D[ cornerIdx ].y/2)
			cv2.circle( img, ( x, y ), 5, ( 255, 0, 0 ), -1 )
			cv2.putText( img, "corner", ( x, y ), cv2.FONT_HERSHEY_COMPLEX, 2, ( 255, 0, 0 ), 2 )