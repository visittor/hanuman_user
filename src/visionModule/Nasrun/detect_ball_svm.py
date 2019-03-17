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

from geometry_msgs.msg import Point32 

from forwardkinematic import getMatrixForForwardKinematic, loadDimensionFromConfig

from colorSegmentation import colorSegmentation, createColorDefFromDict
from scanLine2 import findBoundary, findLinearEqOfFieldBoundary

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

#MODEL_PATH = os.path.join( os.getenv( 'ROS_WS' ), 'src/hanuman_user/config/model/model_with_probability.pkl' )
MODEL_PATH = os.path.join( os.getenv( 'ROS_WS' ), 'src/hanuman_user/config/model/real_model_with_prob.pkl' )

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
		robotConfigPathStr = rospy.get_param( '/robot_config', None )

		if robotConfigPathStr is None:
			raise TypeError( 'Required robot config.' )
		
		#	get model object
		#	positive threshold is 0.70
		self.predictor = HOG_SVM( MODEL_PATH, 0.70 )

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

		#	get image property
		imageWidth = img.shape[ 1 ]
		imageHeight = img.shape[ 0 ]
		
		#	initial final position
		bestPosition = Point32()
		ballErrorList = Point32()
		ballConfidence = 0.0

		#	blur image and change to hsv format
		blurImage = cv2.GaussianBlur( img, ( 5, 5 ), 0 )

		#	segmentation and get marker
		marker = colorSegmentation( blurImage, self.colorDefList )

		#	get field boundery, green color ID is 2
		fieldContour, fieldMask = findBoundary( marker, 2, flip = False )
		fieldContourMiddle = fieldContour[ 1:-1 ].copy()
		
		#
		#	create new mask from ransac
		#
		
		#	get ransac result
		linePropertyAttribute = findLinearEqOfFieldBoundary( fieldContourMiddle )
		
		#	create list of poit
		pointList = list()
		
		for lineCoeff in linePropertyAttribute:
			
			#	get linear coefficient
			x0 = lineCoeff[ 2 ]
			xf = lineCoeff[ 3 ]
			m = lineCoeff[ 0 ]
			c = lineCoeff[ 1 ]
			
			#	calculate y from x
			x = np.arange( x0, xf, dtype = np.int64 )
			y = np.int64( m * x ) + int( c )
			
			contour = np.vstack( ( x, y ) ).transpose()
			countour = contour.reshape( -1, 1, 2 )
			
			pointList.append( countour )
		
		if len( pointList ) > 1:
			contour = np.vstack( pointList )
#			print pointList[ 0 ].shape
#			print pointList[ 1 ].shape
#			print contour.shape
		else:
			contour = pointList[ 0 ]
		
		firstPoint = np.array( [ [ [ 0, img.shape[ 0 ] - 1 ] ] ] )
		lastPoint = np.array( [ [ [ img.shape[ 1 ] - 1, img.shape[ 0 ] - 1 ] ] ] )
		contour = np.concatenate( ( firstPoint, contour, lastPoint ) ) 

		self.contourVis = contour
		
		newFieldMask = np.zeros( marker.shape, dtype = np.uint8 )
		cv2.drawContours( newFieldMask, [ contour ], 0, 1, -1 )
		
		#
		#	find white contour in mask
		#
		
		#	get white object from marker color of white ID is 5
		whiteObject = np.zeros( marker.shape, dtype = np.uint8 )
		whiteObject[ marker == 5 ] = 1

		#	get white object only the field
		#whiteObjectInField = whiteObject * fieldMask.astype( np.uint8 )
		whiteObjectInField = whiteObject * newFieldMask
		whiteObjectInField *= 255

		#	find contour from white object in field
		whiteContours = cv2.findContours( whiteObjectInField, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]
		
		#	extract feature and predict it
		extractStatus = self.predictor.extractFeature( img, whiteContours, objectPointLocation = 'bottom' )
		
		#	check extract status
		if extractStatus == True:
			
			#	predict si wait a rai
			self.predictor.predict()
		
			bestPositionList = self.predictor.getBestRegion()
			
			if len( bestPositionList ) != 0:
				
				#	convert to point32
				bestPosition = Point32( bestPositionList[ 0 ], bestPositionList[ 1 ], 0.0 )
				
				#	get bounding box object not only position
				bestBounding = self.predictor.getBestBoundingBox()
				
				#	get another point of object
				centerX, centerY = bestBounding.calculateObjectPoint( 'center' )
				
				#	calculate error
				errorX, errorY = self.calculateError( imageWidth, imageHeight, centerX, centerY )
				ballErrorList = Point32( errorX, errorY, 0.0 )
				
				#	ball confidence
 				ballConfidence = 1.0
 
		#	define vision message instance
		msg = visionMsg()

		#	assign to message
		msg.object_name = [ 'ball' ]
		msg.pos2D = [ bestPosition ]
		msg.imgH = imageHeight
		msg.imgW = imageWidth
		msg.object_error = [ ballErrorList ]
		msg.object_confidence = [ ballConfidence ]
		msg.header.stamp = rospy.Time.now()

		return msg

	def visualizeFunction(self, img, msg):
		"""For visualization by using cranial nerve monitor"""
		if msg.object_confidence[ 0 ] > 0.9:
			cv2.circle( img, ( msg.pos2D[ 0 ].x, msg.pos2D[ 0 ].y ), 10, ( 255, 0, 0 ), -1 )

		cv2.drawContours( img, [ self.contourVis ], 0, ( 255, 0, 0 ), 1 )
		#cv2.drawContours( img, [ self.contourVis2 ], -1, ( 0, 0, 255 ), 2 )
		
		pass
		
class Kinematic( KinematicModule ):
	
	def __init__( self ):
		
		super( Kinematic, self ).__init__()

		#	load robot parameter via configboj
		robotConfigPathStr = rospy.get_param( "/robot_config", None )
		if robotConfigPathStr is None:
			raise TypeError( "Robot config should not NoneType" )
		config = configobj.ConfigObj( robotConfigPathStr )
		
		#	get camera parameter
		fx = float( config[ "CameraParameters" ][ "fx" ] )
		fy = float( config[ "CameraParameters" ][ "fy" ] ) 
		cx = float( config[ "CameraParameters" ][ "cx" ] ) 
		cy = float( config[ "CameraParameters" ][ "cy" ] ) 

		#	create intrinsic matrix 3x3
		intrinsicMatrix = np.array( [ [ fx,  0,  cx ],
					      [  0, fy,  cy ],
			       		      [  0,  0,   1 ] ] )

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

		#	load dimension from config
		loadDimensionFromConfig( robotConfigPathStr )
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
		
		#	get object name and passthrough
		objectNameList = objMsg.object_name
		
		#	Get camera kinematics
		# transformationMatrix = self.forwardKinematics( joint )
		qPan = getJsPosFromName( joint, "pan" )
		qTilt = getJsPosFromName( joint, "tilt" )
		transformationMatrix = getMatrixForForwardKinematic( qPan, qTilt )
		
		#	initial list
		ball3DCartesianList = list()
		ball2DPolarMsg = list()
		
		#
		#	convert for 'ball' only
		#
		
		#	loop every object to calculate 3D position
		for objIndex in range( len( objMsg.object_name ) ):
			
			if objMsg.object_confidence[ objIndex ] > 0.50:


				#	get 2D ball position
				ballPosition = np.array( [ objMsg.pos2D[ objIndex ].x, 
							   objMsg.pos2D[ objIndex ].y ], dtype = np.float64 )
				ballPosition = ballPosition.reshape( -1, 2 )


				#	get ball in world coordinate
				#	ball3DCartesian is returned in form [ ( 'projection', arrayOfPos ) ] 
				ball3DCartesian = self.calculate3DCoor( ballPosition, 
									HCamera = transformationMatrix )[ 0 ][ 1 ]
				#	get polar coordinate
				#	handle error when 3D coordinate is cannot calculate 3D position
				try:
					r = np.sqrt( ball3DCartesian[ 0 ] ** 2 + ball3DCartesian[ 1 ] ** 2 )
					theta = np.arctan2( ball3DCartesian[ 1 ], ball3DCartesian[ 0 ] )
					ball2DPolar = np.array( [ r, theta ] )

					#	compress to msg
					ball3DCartesianMsg = Point32( ball3DCartesian[ 0 ], 
					     			      ball3DCartesian[ 1 ], 
								      ball3DCartesian[ 2 ] )

					ball2DPolarMsg = Point32( ball2DPolar[ 0 ],
								  ball2DPolar[ 1 ],
						  				 0 )			
				except Exception as e:
					rospy.logwarn( e )
					ball3DCartesianMsg = Point32()
					ball2DPolarMsg = Point32()	
			else:

				ball3DCartesianMsg = Point32()
				ball2DPolarMsg = Point32()
			
		#	If not find the ball
#		if objMsg.ball_confidence == False or len( objMsg.ball ) == 0:
#			
#			#	Set ball 3D Cartesion is None
#			ball3DCartesian = None		
#
#		else:
#			#	Calculate 3D coordinate respect to base frame
#			ballPosition = np.array( objMsg.ball, dtype = np.float64 ).reshape( -1, 2 )
#			ball3DCartesian = self.calculate3DCoor( ballPosition, HCamera = transformationMatrix )
#			ball3DCartesian = ball3DCartesian[ 0 ][ 1 ]
#
#		#	If ball 3D cartesion is None
#		if ball3DCartesian is not None:
#
#			#	Calculate polar coordinate change x, y -> r, theta
#			#ball2DPolar = cv2.cartToPolar( ball3DCartesian[ 0 ], ball3DCartesian[ 1 ] )
#			#ball2DPolar = np.array( [ ball2DPolar[ 0 ][ 0 ], ball2DPolar[ 0 ][ 0 ] ] )
#			r = np.sqrt( ball3DCartesian[ 0 ] ** 2 + ball3DCartesian[ 1 ] ** 2 )
#			theta = np.arctan2( ball3DCartesian[ 1 ], ball3DCartesian[ 0 ] )
#			ball2DPolar = np.array( [ r, theta ] )
#
#		else:
#
#			#	If can't calculate 3D return empty vector
#			ball3DCartesian = np.array( [  ] )
#			ball2DPolar = np.array( [  ] )
#
#		rospy.loginfo( "\nPosition in 3D coordinate : {}\n".format( ball3DCartesian ) )
#		rospy.logdebug( "\nPosition in Polar coordinate : {}\n".format( ball2DPolar ) )
		
	 	#	Get 2D projection back to camera
		self.point2D1 = self.calculate2DCoor( self.points, "ground", HCamera= transformationMatrix )
		self.point2D1 = np.array( self.point2D1 )

 		#	Publist positon dict message
		msg = postDictMsg()
		msg.object_name = objectNameList
		msg.pos3D_cart = [ ball3DCartesianMsg ]
		msg.pos2D_polar = [ ball2DPolarMsg ]
		msg.pos2D = objMsg.pos2D
		msg.imgW = objMsg.imgW
		msg.imgH = objMsg.imgH
		msg.object_error = objMsg.object_error
		msg.object_confidence = objMsg.object_confidence
		msg.header.stamp = rospy.Time.now()

		return msg

	#	Just visualize !
	#	TODO : Clean and refactor later
	def loop(self):
#		blank = np.zeros((480,640,3), dtype=np.uint8)
#		if self.point2D1 is not None:
#			for i in self.pattern:
#				point1 = self.point2D1[i[0]]
#				point2 = self.point2D1[i[1]]
#				# print point1, point2
#				ret, pt1, pt2 = self.clipLine(point1, point2, blank.shape)
#				if ret:
#					# print "Draw"
#					cv2.line(blank, pt1, pt2, (255,0,0), 4)
#		
#		cv2.imshow("img", blank)
#		cv2.waitKey(1)
		pass

	def end( self ):
		cv2.destroyAllWindows()

#	create instance
vision_module = ImageProcessing()
kinematic_module = Kinematic()

