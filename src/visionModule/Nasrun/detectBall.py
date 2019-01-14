#!/usr/bin/env python


from visionManager.visionModule import VisionModule, KinematicModule
from newbie_hanuman.msg import visionMsg, postDictMsg

# from camera_kinematics import CameraKinematic
from forwardkinematic import getMatrixForForwardKinematic, loadDimensionFromConfig

import rospy

import numpy as np
import cv2

import os
import sys

import time

def getJsPosFromName(Js, name):
	'''
	Get Joint state of pantilt.
	argument:
		Js 		:	Joint state message
		name 	:	(str) Either 'pan' or 'tilt'
	return:
		Position of that joint
	'''
	indexJs = Js.name.index(name)
	return Js.position[indexJs]


class ImageProcessing( VisionModule ):

	def __init__( self ):
		super(ImageProcessing, self).__init__()

		#   define message
		self.objectsMsgType = visionMsg

		colorList = [0, 133, 131, 18, 255, 255]

		self.__lower = np.array( colorList[ : 3 ] )
		self.__upper = np.array( colorList[ 3 : 6 ] )

		self.__previousPosition = [ 0., 0. ]

	def ImageProcessingFunction(self, img, header): 

		#	get image property
		imageWidth = img.shape[ 1 ]
		imageHeight = img.shape[ 0 ]

		#   convert to hsv
		hsvImage = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )
		maskImage = cv2.inRange( hsvImage, self.__lower, self.__upper )

		#   find contour
		contours = cv2.findContours( maskImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]  

		if len( contours ) != 0:
			areaList = [ cv2.contourArea( contour ) for contour in contours ]
			maxArea = max( areaList )
			idxContour = areaList.index( maxArea )

			ballContour = contours[ idxContour ]
			
			try:
				#	find image moments
				M = cv2.moments( ballContour )
				# cx = int( M['m10'] / M['m00'] )
				# cy = int( M['m01'] / M['m00'] )
				cx, cy = tuple(ballContour[ballContour[:,:,1].argmax()][0])
				ballPosition = [ int( cx ), int( cy ) ]
				#	calculate error
				#	NOTE : edit error y for switch sign for control motor
				errorX = ( ballPosition[ 0 ] - imageWidth / 2. ) / ( imageWidth / 2. )
				errorY = ( ballPosition[ 1 ] - imageHeight / 2. ) / ( imageHeight / 2. )

				ballError = [ errorX, errorY ]

				isDetectBall = True

			except ZeroDivisionError:
				ballPosition = []
				ballError = list()
				isDetectBall = False

		else:
			ballPosition = []
			ballError = list()
			isDetectBall = False


		msg = visionMsg()
		msg.ball = ballPosition
		msg.imgH = hsvImage.shape[0]
		msg.imgW = hsvImage.shape[1]
		msg.ball_error = ballError
		msg.ball_confidence = isDetectBall
		msg.header.stamp = rospy.Time.now()

		self.__previousPosition = ballPosition

		return msg

	def visualizeFunction(self, img, msg):

		# #	draw circle
		if msg.ball_confidence:
			rospy.logdebug( " error X : {}, error Y : {} ".format( msg.ball_error[ 0 ], msg.ball_error[ 1 ] ) )
			cv2.circle( img, ( msg.ball[ 0 ], msg.ball[ 1 ] ), 10, ( 255, 0, 0 ), -1 )
		
		cv2.circle( img, ( msg.imgW / 2, msg.imgH / 2 ), 5, ( 0, 0, 255 ), -1 )

class Kinematic( KinematicModule ):
	
	def __init__( self ):
		
		super( Kinematic, self ).__init__()

		#	Load intrinsic camera
		#	Path of camera matrix
		intrinsicMatrixPath = '/home/visittor/camMat.npz'
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
		loadDimensionFromConfig( "/home/visittor/ros_ws/src/hanuman_user/script/robotDimension.ini" )

		self.msg = None

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
			ball2DPolar = cv2.cartToPolar( ball3DCartesian[ 0 ], ball3DCartesian[ 1 ] )
			ball2DPolar = np.array( [ ball2DPolar[ 0 ][ 0 ], ball2DPolar[ 0 ][ 0 ] ] )

		else:

			#	If can't calculate 3D return empty vector
			ball3DCartesian = np.array( [  ] )
			ball2DPolar = np.array( [  ] )

		rospy.loginfo( "\nPosition in 3D coordinate : {}\n".format( ball3DCartesian ) )

 		#	Publist positon dict message
		self.msg = postDictMsg()
		self.msg.ball_cart = ball3DCartesian.reshape( -1 )
		self.msg.ball_polar = ball2DPolar.reshape( -1 )
		self.msg.ball_img = objMsg.ball
		self.msg.imgW = objMsg.imgW
		self.msg.imgH = objMsg.imgH
		self.msg.ball_error = ballError
		self.msg.ball_confidence = objMsg.ball_confidence
		self.msg.header.stamp = rospy.Time.now()

		return self.msg

	#	Just visualize !
	#	TODO : Clean and refactor later
	def loop(self):
		
		field = np.zeros( (600, 600, 3), dtype = np.uint8 )
		field[:,:,1] = 255

		for ii in range( 0, 600, 100 ):
			cv2.line( field, (ii, 0), (ii, 600), (0,0,0), 1 )
			cv2.line( field, (0, ii), (600, ii), (0,0,0), 1 )

		pt1 = (300, 550)
		pt2 = (285, 600)
		pt3 = (315, 600)

		triangle_cnt = np.array( [pt1, pt2, pt3] )

		cv2.drawContours(field, [triangle_cnt], 0, (0, 0, 255), -1)

		if self.msg is not None and self.msg.ball_confidence:
			x, y = self.msg.ball_cart[:2]
			x = int( x * 100.0 )
			y = int( y * 100.0 )
			y = 300 - y
			x = 600 - x

			cv2.circle( field, (y,x), 4, (125,125,255), -1 )

		cv2.imshow("img", field)
		cv2.waitKey(1)

	def end( self ):
		cv2.destroyAllWindows()

#	create instance
vision_module = ImageProcessing()
kinematic_module = Kinematic()