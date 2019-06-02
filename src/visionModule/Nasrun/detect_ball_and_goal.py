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

from configobj import ConfigObj

from visionManager.visionModule import VisionModule, KinematicModule
from utility.HanumanForwardKinematic import loadDimensionFromConfig, getMatrixForForwardKinematic
from utility.transformationModule import Project3Dto2D, Project2Dto3D, getInverseHomoMat

from newbie_hanuman.msg import visionMsg, postDictMsg

from geometry_msgs.msg import Point32 

import math

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
		self.predictor = HOG_SVM( FootballModelPath, GoalModelPath, 0.80, rectangleThreshold=30 )
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

		self.cameraMatrix = np.array( [ [fx, 0, cx], [0, fy, cy], [0, 0, 1] ] )

		self.tiltForLookAtFoot = float(self.config['PanTiltPlanner'].get( 'TiltAngleForLookAtFoot', 60 ))
		self.tiltForLookAtFoot = math.radians( self.tiltForLookAtFoot )

	def calculateError( self, imgW, imgH, centerX, centerY ):

		errorX = ( centerX - imgW / 2. ) / ( imgW / 2. )
		errorY = ( centerY - imgH / 2. ) / ( imgH / 2. )

		return errorX, errorY

	def getWhiteObjectContour( self, marker, fieldContour ):

		#   Create mask from new contour
		fieldMask = np.zeros( marker.shape, dtype=np.uint8 )
		cv2.drawContours( fieldMask, [ fieldContour ], 0, 1, -1 )

		whiteObjectMask = np.zeros( marker.shape, dtype=np.uint8 )
		whiteObjectMask[ marker == 5 ] = 1

		whiteObjectInFieldMask = cv2.bitwise_and(whiteObjectMask, fieldMask) * 255
		kernel = np.ones( (5,5), dtype=np.uint8 )
		whiteObjectInFieldMask = cv2.morphologyEx( whiteObjectInFieldMask, cv2.MORPH_OPEN, kernel )
		whiteObjectInFieldMask = cv2.morphologyEx( whiteObjectInFieldMask, cv2.MORPH_CLOSE, kernel )

		whiteObjectContours = cv2.findContours( whiteObjectInFieldMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

		return whiteObjectContours

	def getWhiteObjectContour_lookDown( self, imgW, imgH ):

		yOffset = 80

		centerL = imgW / 4, (imgH - yOffset) / 2
		centerR = (3*imgW )/ 4, (imgH - yOffset) / 2

		w = imgW / 4
		h = ( imgH - yOffset ) / 2

		w = w if h > w else h
		h = h if w > h else w

		cntL = np.array( [  [centerL[0]-w, centerL[1]-h],
							[centerL[0]+w, centerL[1]-h],
							[centerL[0]+w, centerL[1]+h],
							[centerL[0]-w, centerL[1]+h] ] ).astype(int).reshape(-1,1,2)

		cntR = np.array( [  [centerR[0]-w, centerR[1]-h],
							[centerR[0]+w, centerR[1]-h],
							[centerR[0]+w, centerR[1]+h],
							[centerR[0]-w, centerR[1]+h] ] ).astype(int).reshape(-1,1,2)

		return [cntL, cntR]

	def getHorizonLine( self, pan, tilt, dist = 100 ):

		point3D = np.array( [dist, 0.0, 0.0] )

		H = getMatrixForForwardKinematic( pan, tilt )
		H = getInverseHomoMat( H )

		x, horizon = Project3Dto2D( point3D, H, self.cameraMatrix )

		# print Project2Dto3D( np.array( [320, 240]), H, self.cameraMatrix )

		# print x, horizon

		return horizon

	def ImageProcessingFunction( self, img, header, pan_tilt = None ):

		# horizon = self.getHorizonLine( pan_tilt.position[0], pan_tilt.position[1] ) if pan_tilt is not None else 0
		# horizon = self.getHorizonLine( 0, math.radians( 0 ) )

		startTime = time.time()

		objNameList = list()
		pos2DList = list()
		errorList = list()
		confidenceList = list()

		imgH, imgW = img.shape[ :2 ]

		marker = img[:, :, 1]
		gray = img[:, :, 0]

		# horizon = int(min( imgH, max( 0, horizon ) ))

		# marker[ : horizon ] = 0
		# gray[ : horizon ] = 0

		fieldContour, _ = findBoundary( marker, 2 )
		coeff = findLinearEqOfFieldBoundary( fieldContour )

		ransacContours =  [ [0, imgH] ]
		for m, c, x0, xf in coeff:
			ransacContours.append( [ x0, m*x0 + c ] )
			ransacContours.append( [ xf, m*xf + c ] )
		ransacContours.append( [imgW, imgH] )
		ransacContours = np.vstack( ransacContours ).astype(int).reshape(-1,1,2)

		if len( coeff ) > 1:
			#	find y intersect
			xIntersect = coeff[ 0 ][ 3 ]
			m = coeff[ 0 ][ 0 ]
			c = coeff[ 0 ][ 1 ]

			yIntersect = ( m * xIntersect ) + c

			errorX, errorY = self.calculateError( imgW, imgH, xIntersect, yIntersect )
			
			objNameList.append( 'field_corner' )
			pos2DList.append( Point32( x = xIntersect, y = yIntersect, z = 0.0 ) )
			errorList.append( Point32( x = errorX, y = errorY, z = 0.0 ) )
			confidenceList.append( 1.0 )

		self.whiteObjectContours = self.getWhiteObjectContour( marker, ransacContours )
		# self.whiteObjectContours = self.getWhiteObjectContour_lookDown( imgW, imgH )

		canExtract = self.predictor.extractFeature( gray, self.whiteObjectContours, objectPointLocation="bottom" )

		if canExtract:

			self.predictor.predict()

			goalList = self.predictor.getGoal()
			foundBall = self.predictor.getBestRegion()

			if foundBall:

				objNameList.append( 'ball' )

				#	get bounding box object not only position
				bestBounding = self.predictor.getBestBoundingBox()
				#	get another point of object
				botX, botY = bestBounding.bottom
				centerX, centerY = bestBounding.center

				pos2DList.append( Point32( botX,
										   botY,
										   0.0 ) )
				#	calculate error
				errorX, errorY = self.calculateError( imgW, imgH, centerX, centerY )

				errorList.append( Point32( x = errorX, y = errorY, z = 0.0 ) )

				confidenceList.append( bestBounding.footballProbabilityScore )

			for goalObj in goalList:

				objNameList.append( 'goal' )
				botX, botY = goalObj.bottom
				pos2DList.append( Point32( botX,
										   botY,
										   0.0 ) )
				
				errorX, errorY = self.calculateError( imgW, imgH, botX, botY )
				errorList.append( Point32( errorX, errorY, 0.0 ) )

				confidenceList.append( goalObj.goalProbabilityScore )

		msg = self.createVisionMsg( objNameList, pos2DList, errorList, confidenceList, imgW, imgH )
 
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
		super( ImageProcessing, self ).visualizeFunction( img, msg )

		cv2.drawContours( img, self.whiteObjectContours, -1, (0,0,255), 2 )

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

class Kinematic( KinematicModule ):

	def __init__( self ):
		super( Kinematic, self ).__init__( )

		robotConfigPathStr = rospy.get_param( '/robot_config', None )
		config = configobj.ConfigObj( robotConfigPathStr )
		loadDimensionFromConfig( robotConfigPathStr )
		# config = configobj.ConfigObj( '/home/visittor/ros_ws/src/newbie_hanuman/config/robot_sim.ini' )
		# loadDimensionFromConfig( '/home/visittor/ros_ws/src/newbie_hanuman/config/robot_sim.ini' )

		# loadDimensionFromConfig( configPath )
		fx = float( config['CameraParameters']['fx'] )
		fy = float( config['CameraParameters']['fy'] )
		cx = float( config['CameraParameters']['cx'] )
		cy = float( config['CameraParameters']['cy'] )
		imgW = float( config['CameraParameters']['imgW'] )
		imgH = float( config['CameraParameters']['imgH'] )

		self.cameraMatrix = np.array( [ [fx, 0, cx], [0, fy, cy], [0, 0, 1] ] )

		self.set_IntrinsicCameraMatrix( self.cameraMatrix )
		self.add_plane( "ground", np.eye( 4 ),
						(-np.inf, np.inf), (-np.inf, np.inf), (-np.inf, np.inf) )


		self.objectsMsgType =  visionMsg
		self.posDictMsgType = postDictMsg

		self.landmarkName = []
		self.landmarkPose3D = []

		self.points2D = []

		self.subscribeMotorCortex = True

	def kinematicCalculation( self, objMsg, js, cortexMsg, rconfig=None ):

		self.points2D = [ [p.x, p.y] for p in objMsg.pos2D ]

		pitch = math.radians(cortexMsg.pitch) if cortexMsg is not None else 0.0
		roll = math.radians(cortexMsg.roll)	if cortexMsg is not None else 0.0
		# print math.degrees(pitch), math.degrees(roll)
		# roll = pitch = 0.0
		tranvec = np.zeros( (3,1) )
		rotvec = np.array( [roll, pitch, 0.0] )

		HRotate = self.create_transformationMatrix(tranvec, rotvec, 
													'rpy', order="tran-first")
		
		H = getMatrixForForwardKinematic( js.position[0], js.position[1], roll, pitch )
		H = np.matmul( HRotate, H )

		points3D = self.calculate3DCoor( self.points2D, HCamera = H )

		polarList = []
		cartList = []
		names = []
		confidences = []
		errorList = []
		pos2DList = []

		imgH = objMsg.imgH
		imgW = objMsg.imgW

		for (plane, p3D), name, confidence, p2D, err in zip(points3D, objMsg.object_name, objMsg.object_confidence, objMsg.pos2D, objMsg.object_error ):
			if plane is None:
				pass

			else:
				x, y = p3D[:2]
				
				cartList.append( Point32( x = x, y = y ) )

				rho = math.sqrt( x**2 + y**2 )
				phi = math.atan2( y, x )

				polarList.append( Point32( x = rho, y = phi) )

				names.append( name )
				confidences.append( confidence )
				errorList.append( Point32( x = err.x, y = err.y ) )
				pos2DList.append( p2D )
		# print points3D
		msg = postDictMsg( )
		msg.object_name = names
		msg.pos3D_cart = cartList
		msg.pos2D_polar = polarList
		msg.pos2D = pos2DList
		msg.object_error = errorList
		msg.object_confidence = confidences
		msg.imgH = imgH
		msg.imgW = imgW

		return msg