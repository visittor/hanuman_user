from visionManager.visionModule import KinematicModule, VisionModule

from newbie_hanuman.msg import postDictMsg
from geometry_msgs.msg import Point32

from utility.HanumanForwardKinematic import *

from scanLine2 import findBoundary, findLinearEqOfFieldBoundary, findNewLineFromRansac
from imageProcessingModule import findGoal
from imageProcessingModule.mobile_robot_project import findPole
from imageProcessingModule.hog_svm import HOG_SVM

from colorSegmentation import colorSegmentation, createColorDefFromDict

from newbie_hanuman.msg import visionMsg, postDictMsg

import numpy as np 
import cv2

import rospy

import math

import configobj

import os

import time

FootballModelPath =  os.path.join( os.getenv( 'ROS_WS' ), "src/hanuman_user/config/model/real_model_with_prob.pkl" )
GoalModelPath = os.path.join( os.getenv( 'ROS_WS' ), "src/hanuman_user/config/model/model_goal_gray.pkl" ) 

class Vision( VisionModule ):

	def __init__( self ):

		super( Vision, self ).__init__( )

		self.objectsMsgType = visionMsg

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

	def _setColorConfig( self, colorConfig ):

		super( Vision, self )._setColorConfig( colorConfig )

		self.greenID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'green' ][ 0 ]
		self.whiteID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'white' ][ 0 ]
		self.orangeID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'orange' ][ 0 ]
		self.blueID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'blue' ][ 0 ]
		self.yellowID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'yellow' ][ 0 ]
		self.magentaID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'magenta' ][ 0 ]

	def calculateError( self, imageWidth, imageHeight, centerX, centerY ):

		errorX = ( centerX - imageWidth / 2. ) / ( imageWidth / 2. )
		errorY = ( centerY - imageHeight / 2. ) / ( imageHeight / 2. )

		return errorX, errorY

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
		ransac = findNewLineFromRansac( fieldContour, imageWidth, imageHeight )

		if len( ransac ) > 0:

			ransacContours, coeff = ransac[0], ransac[1] 
		else:
			ransacContours, coeff = fieldContour, []

		if len( coeff ) > 1:
			#	find y intersect
			xIntersect = coeff[ 0 ][ 3 ]
			m = coeff[ 0 ][ 0 ]
			c = coeff[ 0 ][ 1 ]

			yIntersect = ( m * xIntersect ) + c
			
			
			intersectPoint = Point32( x = xIntersect, y = yIntersect, z = 0.0 )
			errorX, errorY = self.calculateError( imageWidth, imageHeight, xIntersect, yIntersect )
			
			errorIntersectPoint = Point32( x = errorX, y = errorY, z = 0.0 )
			
			objNameList.append( 'field_corner' )
			pos2DList.append( intersectPoint )
			errorList.append( errorIntersectPoint )
			confidenceList.append( 1.0 )

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

		poles = findPole( marker, self.orangeID, self.blueID, self.yellowID, self.magentaID, mask = fieldMask )

		for n, center in poles.items( ):
			for x,y in center:
				objNameList.append( n )
				pos2DList.append( Point32( x=x, y=y ) )

				errorX, errorY = self.calculateError( imageWidth, imageHeight, x, y )

				errorList.append( Point32( x=errorX, y=errorY ) )

				confidenceList.append( 1.0 )

		msg = self.createVisionMsg( objNameList, pos2DList, errorList, confidenceList, imageWidth, imageHeight )
 
		rospy.logdebug( "Time usage : {}".format( time.time() - startTime ) )

		return msg

	def visualizeFunction( self, img, msg ):

		super( Vision, self ).visualizeFunction( img, msg )

		for n, p in zip( msg.object_name, msg.pos2D ):

			x, y = int(p.x), int(p.y)

			if n == 'pole':
				color = ( 255, 0, 0 )

			else:
				color = ( 0, 0, 0 )

			cv2.circle( img, (x,y), 5, color, -1 )

class Kinematic( KinematicModule ):

	def __init__( self ):

		super( Kinematic, self ).__init__( )

		config = configobj.ConfigObj( '/home/visittor/humanoid_data/test_2.ini' )
		loadDimensionFromConfig( '/home/visittor/humanoid_data/test_2.ini' )

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

		for (plane, p3D), name, confidence, p2D in zip(points3D, objMsg.object_name, objMsg.object_confidence, objMsg.pos2D ):
			if plane is None:
				pass

			else:
				x, y = p3D[:2]

				if math.sqrt( x**2 + y**2 ) > 6:
					continue
				
				cartList.append( Point32( x = x, y = y ) )

				rho = math.sqrt( x**2 + y**2 )
				phi = math.atan2( y, x )

				polarList.append( Point32( x = rho, y = phi) )

				names.append( name )
				confidences.append( confidence )
				errorList.append( Point32( x = p2D.x / imgW, y = p2D.y / imgH ) )
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

vision_module = Vision( )
kinematic_module = Kinematic( )