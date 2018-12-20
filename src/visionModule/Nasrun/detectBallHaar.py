#!/usr/bin/env python


from visionManager.visionModule import VisionModule, KinematicModule
from newbie_hanuman.msg import visionMsg, postDictMsg

from camera_kinematics import CameraKinematic

from colorSegmentation import colorSegmentation, waterShed

from scanLine2 import findBoundary

import rospy

import numpy as np
import cv2

import os
import sys

import time


#	GLOBAL VARIABLE
ROS_WORKSPACE = os.getenv( 'ROS_WORKSPACE' )

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

def getFieldMask( image, lower, upper ):	

	#	convert to hsv space
	hsvImage = cv2.cvtColor( image, cv2.COLOR_BGR2HSV )

	#	smoothen image
	hsvImage = cv2.blur( hsvImage, ( 5, 5 ) )

	#	find in-range
	greenMask = cv2.inRange( hsvImage, np.array( lower ), np.array( upper ) )

	#	find contours of field
	contours = cv2.findContours( greenMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

	#	create black image
	fieldMask = np.zeros( greenMask.shape, dtype = np.uint8 )

	#	create field mask
	cv2.drawContours( fieldMask, contours, -1, ( 255, 255, 255 ), -1 )

	return fieldMask


class ImageProcessing( VisionModule ):

	def __init__( self ):
		super(ImageProcessing, self).__init__()

		#   define message
		self.objectsMsgType = visionMsg

		#   get path of model
		self.modelPath = ROS_WORKSPACE + '/src/hanuman_user/model/data_haar111217_2.xml'

		#	get instance of haar cascade
		self.classifier = cv2.CascadeClassifier( self.modelPath )

		greenBoundary = [ 38, 34, 7, 65, 255, 255 ]

		#	lower and upper boundary
		self.upper = greenBoundary[ 3 : 6 ]
		self.lower = greenBoundary[ 0 : 3 ]

	def detectCircular( self, contours ):
		"""
		Detect circular 
		argument :
			contour : ( list )
		"""

		listCircularContour = list()

		for cnt in contours:

			peri = cv2.arcLength( cnt, True )
			approx = cv2.approxPolyDP( cnt, peri * 0.04, True )

			area = cv2.contourArea(cnt)
			hull = cv2.convexHull(cnt)
			hull_area = cv2.contourArea(hull)
			solidity = float(area)/hull_area if hull_area != 0 else 0

			if len( approx ) >= 5 and solidity > 0.5:
				listCircularContour.append( cnt )

		return listCircularContour

	def detectBall( self, whiteObjectList, image ):
		"""
		Detect ball from list of white object
			arguments : 
				whiteObjectList : (list)
				image : (np.array) grayImage
		"""  

		for whiteObject in whiteObjectList:

			#	get bounding rectangular for image roi
			x, y, w, h = cv2.boundingRect( whiteObject )   
 			ballCandidate = image[ y : y+h, x : x+w ]
 			
 			#	classify by haar
 			footballs = self.classifier.detectMultiScale( ballCandidate, 1.3, 3 )

 			if len( footballs ) > 0:
 				return x, y, w, h

 		return list()

 	def findBallError( self, x, y, width, height ):

		errorX = ( x - width / 2. ) / ( width / 2. )
		errorY = ( y - height / 2. ) / ( height / 2. )

		return errorX, errorY

	def ImageProcessingFunction(self, img, header): 

		#	get image property
		imageWidth = img.shape[ 1 ]
		imageHeight = img.shape[ 0 ]

		# #	get field mask
		# fieldMask = getFieldMask( img, self.lower, self.upper )

		# #	convert image to gray scale
		# grayImage = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )

		# #	and with gray color
		# grayImage = cv2.bitwise_and( grayImage, grayImage, mask = fieldMask )

		# #	detect ball
		# footballPositions = self.classifier.detectMultiScale( grayImage, 1.1, 5 )

		# #	check ball
		# if len( footballPositions ) > 0:
		# 	x, y, w, h = footballPositions[ 0 ]
		# 	ballPosition = [ x+w/2, y+h/2 ]
		# 	isBallDetection = True
		# 	ballError = self.findBallError( ballPosition[ 0 ], ballPosition[ 1 ], imageWidth, imageHeight )
		# else:
		# 	ballPosition = list()
		# 	isBallDetection = False
		# 	ballError = list()


		#	get color map and grey color
		grayImage = img[ :, :, 0 ]
		colorMap = img[ :, :, 1 ]

		#	get field boundary
		fieldBoundary, fieldMask = findBoundary( colorMap, 1, flip = False )

		#	get rid off region outside field boundary
		# colorMap *= fieldMask

		marker = np.ones( grayImage.shape, dtype = np.uint8 )
		marker[ colorMap == 1 ] = 0

		grayImage *= fieldMask

		# whiteContours = cv2.findContours( marker, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ] 

		# circularWhiteList = self.detectCircular( whiteContours )

		#footballs = self.detectBall( circularWhiteList, grayImage )
		footballs = self.classifier.detectMultiScale( grayImage, 1.1, 3 )
	
		if len( footballs ) > 0:
			x, y, h, w = footballs[ 0 ]
			ballPosition = [ x+w/2, y+h/2 ]
			isBallDetection = True
			ballError = self.findBallError( ballPosition[ 0 ], ballPosition[ 1 ], imageWidth, imageHeight )
		else:
			ballPosition = list()
			isBallDetection = False
			ballError = list()

		# #	find contour with marker
		# whiteConturs = cv2.findContours( marker, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]  

		# #	find circular white object
		# self.circularWhiteList = self.detectCircular( whiteConturs )

		# #	detect ball
		# footballs = self.detectBall( self.circularWhiteList, grayImage )
		# print footballs
		# if len( footballs ) > 0:
		# 	ballPosition = footballs[ 0 ][ 0 : 2 ]
		# 	isBallDetection = True
		# 	ballError = self.findBallError( ballPosition[ 0 ], ballPosition[ 1 ], imageWidth, imageHeight )
		# else:
		# 	ballPosition = list()
		# 	isBallDetection = False
		# 	ballError = list()
  

		msg = visionMsg()
		msg.ball = ballPosition
		msg.imgH = grayImage.shape[0]
		msg.imgW = grayImage.shape[1]
		msg.ball_error = ballError
		msg.ball_confidence = isBallDetection
		msg.header.stamp = rospy.Time.now()

		return msg

	def visualizeFunction(self, img, msg):

		# #	Render
		# colorImage = np.zeros( img.shape, dtype = np.uint8 )
		# # colorImage[ img[:,:,1] == 1 ] = [0, 128, 0]
		# colorImage[ img[:,:,1] == 3 ] = [0, 128, 255]
		# colorImage[ img[:,:,1] == 8 ] = [255, 255, 255]

		# #	marker
		# marker = np.zeros( img[ :, :, 0 ].shape, dtype = np.uint8 )
		# marker[ img[:,:,1] == 8 ] = 1

		#	find boundary
		fieldBoundary, fieldMask = findBoundary( img[ :, :, 1 ], 1, flip = False )
	
		# #	find circular shape
		# whiteContour = self.detectCircular( whiteContours )

		# cv2.drawContours( colorImage, whiteContour, -1, ( 0, 0, 255 ), 3 )

		# #	draw region boundary of field

		# img[ :, :, : ] = colorImage.copy()

		colorImage = np.zeros( img.shape, dtype = np.uint8 )
		colorImage[ img[:,:,1] == 3 ] = [0, 128, 255]
		colorImage[ img[:,:,1] == 8 ] = [255, 255, 255]
		colorImage[ img[:,:,1] == 1 ] = [0, 128, 0]

		colorImage[ :, :, 0 ] *= fieldMask
	
		cv2.drawContours( colorImage, [ fieldBoundary ], 0, ( 255, 0, 0 ), 2 ) 

		img[ :, :, : ] = colorImage.copy()

		if msg.ball_confidence:

			rospy.logdebug( " error X : {}, error Y : {} ".format( msg.ball_error[ 0 ], msg.ball_error[ 1 ] ) )
			cv2.circle( img, ( msg.ball[ 0 ], msg.ball[ 1 ] ), 10, ( 255, 0, 0 ), -1 )
		
		cv2.circle( img, ( msg.imgW / 2, msg.imgH / 2 ), 5, ( 0, 0, 255 ), -1 )


class Kinematic( KinematicModule ):
	
	def __init__( self ):
		
		super( Kinematic, self ).__init__()

		#	Load intrinsic camera
		#	Path of camera matrix


		intrinsicMatrixPath = ROS_WORKSPACE + '/src/hanuman_user/camera_matrix/camera_matrix.npy'
		intrinsicMatrix = np.load( intrinsicMatrixPath )

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
		self.cameraKinematic = CameraKinematic( 4, 0, 45, 2 )


		#	
		#	For visualize
		#

		x, y = np.indices((5,5))
		z = np.zeros((25,1), float)
		points = np.hstack((x.reshape(-1,1), y.reshape(-1,1))).astype(float)
		points[:,0] *= 0.25
		points[:,1] = 0.25*points[:,1] - 0.5
		points = np.hstack((points,z))
		self.points = points.copy()*100

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

	def forwardKinematics( self, jointState ):
		"""
			Calculate forward kinematics from base frame
			return:
				homogenousMatrix = Homogenous transformation matrix from base frame 
				to camaraframe
		"""
		#	get joint state from pan and tilt
		qPan = getJsPosFromName( jointState, "pan" )
		qTilt = getJsPosFromName( jointState, "tilt" )

		#	Calculate transformation matrix
		transformationMatrix = self.cameraKinematic.updateMatrix( qPan, qTilt )

		return transformationMatrix

	def kinematicCalculation( self, objMsg, joint ):
		
		#	Get ball error
		ballError = objMsg.ball_error

		#	Get camera kinematics
		transformationMatrix = self.forwardKinematics( joint )

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

		rospy.logdebug( "\nPosition in 3D coordinate : {}\n".format( ball3DCartesian ) )

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
		# blank = np.zeros((480,640,3), dtype=np.uint8)
		# if self.point2D1 is not None:
		# 	for i in self.pattern:
		# 		point1 = self.point2D1[i[0]]
		# 		point2 = self.point2D1[i[1]]
		# 		# print point1, point2
		# 		ret, pt1, pt2 = self.clipLine(point1, point2, blank.shape)
		# 		if ret:
		# 			# print "Draw"
		# 			cv2.line(blank, pt1, pt2, (255,0,0), 4)
		
		# cv2.imshow("img", blank)
		# cv2.waitKey(1)
		pass

	def end( self ):
		#cv2.destroyAllWindows()
		pass

#	create instance
vision_module = ImageProcessing()
kinematic_module = Kinematic()