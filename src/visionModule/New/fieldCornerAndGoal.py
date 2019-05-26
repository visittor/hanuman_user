from visionManager.visionModule import KinematicModule, VisionModule

from newbie_hanuman.msg import postDictMsg
from geometry_msgs.msg import Point32

from utility.HanumanForwardKinematic import *

from scanLine2 import findBoundary, findLinearEqOfFieldBoundary, findNewLineFromRansac
from imageProcessingModule import findGoal

import numpy as np 
import cv2

import configobj

class Vision( VisionModule ):

	def __init__( self ):
		super( Vision, self ).__init__( )
		self.objectsMsgType = postDictMsg

	def _setColorConfig( self, colorConfig ):

		super( Vision, self )._setColorConfig( colorConfig )

		self.greenID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'green' ][ 0 ]
		self.whiteID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'white' ][ 0 ]

		self.boundaryLine = []
		self.fieldContour = []

	def ImageProcessingFunction( self, img, header ):

		color_map = img[:, :, 1].copy()

		self.fieldContour, fieldMask = findBoundary( color_map, self.greenID )

		self.boundaryLine = findLinearEqOfFieldBoundary( self.fieldContour[1:-1] )

		newFieldContour = findNewLineFromRansac( self.fieldContour, 640, 480 )
		goalList = findGoal( newFieldContour, color_map, goalColorID = self.greenID )

		name = []
		pos2D = []
		confidence = []
		imgH = color_map.shape[0]
		imgW = color_map.shape[1]

		if len(self.boundaryLine) == 2:
			intersect_x = self.boundaryLine[0][3]
			intersect_y = self.boundaryLine[0][0]*intersect_x + self.boundaryLine[0][1]

			name.append( 'field_corner' )
			pos2D.append( Point32( x = intersect_x, y = intersect_y ) )
			confidence.append( 0.8 )

		for p in goalList:
			if p is None:
				break

			x,y = p

			name.append( 'goal' )
			pos2D.append( Point32( x = x, y = y ) )
			confidence.append( 0.8 )

		msg = postDictMsg( )
		msg.object_name = name
		msg.pos2D = pos2D
		msg.object_confidence = confidence
		msg.imgH = imgH
		msg.imgW = imgW

		msg.header = header

		return msg

	def visualizeFunction( self, img, msg ):

		super( Vision, self ).visualizeFunction( img, msg )

		for n, p in zip( msg.object_name, msg.pos2D ):

			x, y = int(p.x), int(p.y)

			if n == 'goal':
				color = (0, 255, 255)

			elif n == 'field_corner':
				color = (255, 0, 255)

			else:
				color = (0,0,0)

			cv2.circle( img, (x,y), 5, color, -1 )

		cv2.drawContours(img, self.fieldContour, -1, (0,255,0), 3)

		color = [ (0,0,255), (255,0,0)]
		for i, (m, c, x1, x2) in enumerate(self.boundaryLine):
			y1 = int(m*x1 + c)
			y2 = int(m*x2 + c)

			x1 = int( x1 )
			x2 = int( x2 )
			cv2.line( img, (x1, y1), (x2, y2), color[i], 3 )

class Kinematic( KinematicModule ):

	def __init__( self ):
		super( Kinematic, self ).__init__( )

		config = configobj.ConfigObj( '/home/visittor/humanoid_data/test_2.ini' )
		loadDimensionFromConfig( '/home/visittor/humanoid_data/test_2.ini' )
		print getRobotConfiguration( )
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


		self.objectsMsgType =  postDictMsg
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