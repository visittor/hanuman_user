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

from visionManager.visionModule import VisionModule, KinematicModule
from newbie_hanuman.msg import visionMsg, postDictMsg

from forwardkinematic import getMatrixForForwardKinematic, loadDimensionFromConfig

from colorSegmentation import colorSegmentation, createColorDefFromDict
from scanLine2 import findBoundary

from hog_svm import HOG_SVM

import rospy

import numpy as np
import cv2

import configobj

import time

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

	def __init__( self ):
		super( ImageProcessing, self ).__init__()

		#   define message
		self.objectsMsgType = visionMsg

		#	get color config file path from rosparam
		colorConfigPathStr = rospy.get_param( '/vision_manager/colorConfig', None )

		if colorConfigPathStr is None:
			raise TypeError( 'Required color config.' )

		modelPathStr = rospy.get_param( '/vision_manager/modelPath', None )

		if modelPathStr is None:
			raise TypeError( 'Required trained model' )
		
		#	get model object
		self.predictor = HOG_SVM( modelPathStr )

		#	get color definition from color config ( get only values )
		colorConfigList = configobj.ConfigObj( colorConfigPathStr )
		colorConfigList = colorConfigList.values()

		#	create color definition for using segmentation 
		self.colorDefList = createColorDefFromDict( colorConfigList )

	def calculateError( self, imageWidth, imageHeight, centerX, centerY ):

		errorX = ( centerX - imageWidth / 2. ) / ( imageWidth / 2. )
		errorY = ( centerY - imageHeight / 2. ) / ( imageHeight / 2. )

		return errorX, errorY
		
	def ImageProcessingFunction( self, img, header ): 

		#	get image property
		imageWidth = img.shape[ 1 ]
		imageHeight = img.shape[ 0 ]

		#	blur image and change to hsv format
		blurImage = cv2.blur( img, ( 5, 5 ) )

		#	segmentation and get marker
		marker = colorSegmentation( blurImage, self.colorDefList )

		#	get field boundery, green color ID is 2
		fieldContour, fieldMask = findBoundary( marker, 2, flip = False )
		
		self.contourVis = fieldContour

		#	get white object from marker color of white ID is 5
		whiteObject = np.zeros( marker.shape, dtype = np.uint8 )
		whiteObject[ marker == 5 ] = 1

		#	get white object only the field
		whiteObjectInField = whiteObject * fieldMask.astype( np.uint8 )
		whiteObjectInField *= 255

		#	find contour from white object in field
		whiteContours = cv2.findContours( whiteObjectInField, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]
		
		#	extract feature and predict it
		self.predictor.extractFeature( img, whiteContours, objectPointLocation = 'bottom' )
		self.predictor.predict()

		#	check white contours
		if len( self.predictor.boundingBoxListObject.boundingBoxList ) != 0:
			
			#	choose best region that should be a ball
			self.predictor.chooseBestRegion()

			#	get ball position
			ballPositionList = list( self.predictor.boundingBoxListObject.previousBoundingBox.object2DPosTuple )
			
			print ballPositionList

			#	calculate error
			errorX, errorY = self.calculateError( imageWidth, imageHeight, 
										          ballPositionList[ 0 ], ballPositionList[ 1 ] )
			ballErrorList = [ errorX, errorY ]

			#	assign logic to brain
			isDetectBall = True
		
		else:

			#	send empty list and set false
			ballPositionList = list()
			ballErrorList = list()
			isDetectBall = False


		#	define vision message instance
		msg = visionMsg()

		#	assign to message
		msg.ball = ballPositionList
		msg.imgH = imageHeight
		msg.imgW = imageWidth
		msg.ball_error = ballErrorList
		msg.ball_confidence = isDetectBall
		msg.header.stamp = rospy.Time.now()

		return msg

	def visualizeFunction(self, img, msg):
		"""For visualization by using cranial nerve monitor"""
		if msg.ball_confidence == True:
			cv2.circle( img, tuple( msg.ball ), 10, ( 255, 0, 0 ), -1 )

		cv2.drawContours( img, [ self.contourVis ], -1, ( 255, 0, 0 ), 1 )
		#cv2.drawContours( img, [ self.contourVis2 ], -1, ( 0, 0, 255 ), 2 )
class Kinematic( KinematicModule ):
	
	def __init__( self ):
		
		super( Kinematic, self ).__init__()

		#	Load intrinsic camera
		#	Path of camera matrix
		#intrinsicMatrixPath = '/home/neverholiday/work/ros_ws/src/hanuman_user/config/camera_matrix/camMat.npz'
		intrinsicMatrixPath = rospy.get_param( '/robot_kinematic/intrinsicCameraPath', None )

		if intrinsicMatrixPath is None:
			raise TypeError( "Intrinsic camera matrix should not NoneType" )
		intrinsicMatrix = np.load( intrinsicMatrixPath )[ 'cameraMatrix' ]

		#	Set camera matrix
		self.set_IntrinsicCameraMatrix( intrinsicMatrix )

		#	Define object type 
		self.objectsMsgType = visionMsg
		self.posDictMsgType = postDictMsg

		#	Define translation and rotation for create homogenous transformation of plane
		tranVec = np.array( [ 0, 0, 0 ], float )
		rotVec = np.array( [ 0, 0, 0 ], float )
		self.grounPlaneTransformMatrix = self.create_transformationMatrix( tranVec, rotVec, 'zyz' )

		#	Add plane
		#	TODO : I'm not sure what boundary is, ask new later
		self.add_plane(	"ground", self.grounPlaneTransformMatrix, 
						( -np.inf, np.inf ), ( -np.inf, np.inf ), ( -1, 1 ))

		#	Create instance of camera kinematics
		# self.cameraKinematic = CameraKinematic( 4, 0, 45, 2 )
		robotDimensionPath = rospy.get_param( '/robot_kinematic/robotDimension', None )

		if robotDimensionPath is None:
			raise TypeError( "Robot dimension shold not NoneType" )
		loadDimensionFromConfig( robotDimensionPath )
		#loadDimensionFromConfig( "/home/neverholiday/work/ros_ws/src/hanuman_user/script/robotDimension.ini" )


		#	
		#	For visualize
		#

		x, y = np.indices((5,5))
		z = np.zeros((25,1), float)
		points = np.hstack((x.reshape(-1,1), y.reshape(-1,1))).astype(float)
		points[:,0] *= 0.25
		points[:,1] = 0.25*points[:,1] - 0.5
		points = np.hstack((points,z))
		self.points = points.copy()

		self.point2D1 = None

		self.pattern = [	[0, 4],
							[5, 9],
							[10, 14],
							[15, 19],
							[20, 24],
							[0, 20],
							[1, 21],
							[2, 22],
							[3, 23],
							[4, 24]
						]

		self.worldCoors = list()
		self.labels = list()
		self.rects = list()

	#	TODO : Not sure what it is ?
	def __rect2CntAndCenter(self, rect):
		cnt = np.zeros((4,2), dtype = float)
		center = np.zeros((2), dtype = float)
		for i,p in enumerate(rect.points):
			cnt[i,0] = p.x
			cnt[i,1] = p.y
			center[0] += 0.25*p.x
			center[1] += 0.25*p.y
		return cnt, center

	#	TODO : Not again.
	def clipLine(self, point1, point2, shape):
		if point1 is None or point2 is None:
			return False, point1, point2

		if (-1000000>=point1).any() or (point1>=1000000).any():
			return False, point1, point2

		if (-1000000>=point2).any() or (point2>=1000000).any():
			return False, point1, point2

		point1 = point1.astype(int)
		point2 = point2.astype(int)
		ret, pt1, pt2 = cv2.clipLine( (0,0,shape[1],shape[0]), tuple(point1), tuple(point2))
		# print ret
		return ret, pt1, pt2

	# def forwardKinematics( self, jointState ):
	# 	"""
	# 		Calculate forward kinematics from base frame
	# 		return:
	# 			homogenousMatrix = Homogenous transformation matrix from base frame 
	# 			to camaraframe
	# 	"""
	# 	#	get joint state from pan and tilt
	# 	qPan = getJsPosFromName( jointState, "pan" )
	# 	qTilt = getJsPosFromName( jointState, "tilt" )

	# 	#	Calculate transformation matrix
	# 	transformationMatrix = self.cameraKinematic.updateMatrix( qPan, qTilt )

	# 	return transformationMatrix

	def kinematicCalculation( self, objMsg, joint ):
		
		#	Get ball error
		ballError = objMsg.ball_error

		#	Get camera kinematics
		# transformationMatrix = self.forwardKinematics( joint )
		qPan = getJsPosFromName( joint, "pan" )
		qTilt = getJsPosFromName( joint, "tilt" )
		transformationMatrix = getMatrixForForwardKinematic( qPan, qTilt )

		#	If not find the ball
		if objMsg.ball_confidence == False:
			
			#	Set ball 3D Cartesion is None
			ball3DCartesian = None		

		else:
			#	Calculate 3D coordinate respect to base frame
			ballPosition = np.array( objMsg.ball, dtype = np.float64 ).reshape( -1, 2 )
			ball3DCartesian = self.calculate3DCoor( ballPosition, HCamera = transformationMatrix )
			ball3DCartesian = ball3DCartesian[ 0 ][ 1 ]

		#	If ball 3D cartesion is None
		if ball3DCartesian is not None:

			#	Calculate polar coordinate change x, y -> r, theta
			#ball2DPolar = cv2.cartToPolar( ball3DCartesian[ 0 ], ball3DCartesian[ 1 ] )
			#ball2DPolar = np.array( [ ball2DPolar[ 0 ][ 0 ], ball2DPolar[ 0 ][ 0 ] ] )
			r = np.sqrt( ball3DCartesian[ 0 ] ** 2 + ball3DCartesian[ 1 ] ** 2 )
			theta = np.arctan2( ball3DCartesian[ 1 ], ball3DCartesian[ 0 ] )
			ball2DPolar = np.array( [ r, theta ] )

		else:

			#	If can't calculate 3D return empty vector
			ball3DCartesian = np.array( [  ] )
			ball2DPolar = np.array( [  ] )

		rospy.loginfo( "\nPosition in 3D coordinate : {}\n".format( ball3DCartesian ) )
		rospy.logdebug( "\nPosition in Polar coordinate : {}\n".format( ball2DPolar ) )

		#	Get 2D projection back to camera
		self.point2D1 = self.calculate2DCoor( self.points, "ground", HCamera= transformationMatrix )
		self.point2D1 = np.array( self.point2D1 )

 		#	Publist positon dict message
		msg = postDictMsg()
		msg.ball_cart = ball3DCartesian.reshape( -1 )
		msg.ball_polar = ball2DPolar.reshape( -1 )
		msg.ball_img = objMsg.ball
		msg.imgW = objMsg.imgW
		msg.imgH = objMsg.imgH
		msg.ball_error = ballError
		msg.ball_confidence = objMsg.ball_confidence
		msg.header.stamp = rospy.Time.now()

		return msg

	#	Just visualize !
	#	TODO : Clean and refactor later
	def loop(self):
		blank = np.zeros((480,640,3), dtype=np.uint8)
		if self.point2D1 is not None:
			for i in self.pattern:
				point1 = self.point2D1[i[0]]
				point2 = self.point2D1[i[1]]
				# print point1, point2
				ret, pt1, pt2 = self.clipLine(point1, point2, blank.shape)
				if ret:
					# print "Draw"
					cv2.line(blank, pt1, pt2, (255,0,0), 4)
		
		cv2.imshow("img", blank)
		cv2.waitKey(1)

	def end( self ):
		cv2.destroyAllWindows()

#	create instance
vision_module = ImageProcessing()
kinematic_module = Kinematic()

