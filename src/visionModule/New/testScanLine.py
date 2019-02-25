from visionManager.visionModule import KinematicModule, VisionModule

from std_msgs.msg import Empty

from newbie_hanuman.msg import scanlinePointClound
from newbie_hanuman.msg import point2D

from utility.HanumanForwardKinematic import *
from scanLine2 import findBoundary, findChangeOfColor

import numpy as np 
import cv2

DIMENSION_FOR_SIMULATION = [ 0.49088, 0.0, 0.0205, 0.01075, 0.046, 0.034, -0.002, 0.0 ]
DIMENSION_FOR_REAL_WORLD = [ 0.42889, 0.0055, 0.00187, 0.0254, 0.05, 0.0, 0.0, 14.34]

# FX = 381.36246688113556
# FY = 381.36246688113556
FX = 577.55352783
FY = 580.48583984

# setNewRobotConfiguration( *DIMENSION_FOR_SIMULATION )

class Vision( VisionModule ):

	def __init__( self ):
		super( Vision, self ).__init__( )
		self.objectsMsgType = scanlinePointClound

	def _setColorConfig( self, colorConfig ):

		super( Vision, self )._setColorConfig( colorConfig )

		self.greenID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'green' ][ 0 ]
		self.whiteID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'white' ][ 0 ]

	def ImageProcessingFunction( self, img, header ):

		color_map = img[:, :, 1].copy()
		fieldContour, fieldMask = findBoundary( color_map, 2 )
		pointClound = findChangeOfColor( color_map, self.whiteID, self.greenID, mask=fieldMask, step = 40 )

		points = []
		splitIndexes = []
		
		for scanline in pointClound:

			for x,y in scanline:
				points.append( point2D( x = x, y = y ) )

			splitIndexes.append( len( points ) )

		msg = scanlinePointClound( )
		msg.num_scanline = 8
		msg.min_range = 0
		msg.max_range = 2**16 - 1
		msg.range = []
		msg.points = points
		msg.splitting_index = splitIndexes
		msg.header = header

		return msg

	def visualizeFunction( self, img, msg ):

		super( Vision, self ).visualizeFunction( img, msg )

		for point in msg.points:
			x = int( point.x )
			y = int( point.y )
			cv2.circle(img,(x,y), 4, (0,0,0), -1)
			cv2.circle(img,(x,y), 3, (0,0,255), -1)

class Kinematic( KinematicModule ):

	def __init__( self ):
		super( Kinematic, self ).__init__(  )

		config = configobj.ConfigObj( '/home/visittor/humanoid_data/robotConfig.ini' )
		loadDimensionFromConfig( '/home/visittor/humanoid_data/robotConfig.ini' )

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

		self.objectsMsgType = scanlinePointClound
		self.posDictMsgType = Empty

		self.points3D = []

	def kinematicCalculation( self, objMsg, js, rconfig=None ):

		points2DArray = [ [p.x, p.y] for p in objMsg.points ]
		# js.position[1] *= -1
		# print js.position
		H = getMatrixForForwardKinematic( *js.position )

		points3D = self.calculate3DCoor( points2DArray, HCamera = H )

		ranges = []
		points = []
		for plane, p3D in points3D:
			if plane is None:
				ranges.append( -1 )

			else:
				x, y = p3D[:2]
				ranges.append( np.sqrt( x**2 + y**2 ) )
				points.append( point2D( x = x, y = y ) )

		msg = scanlinePointClound( )
		msg.num_scanline = objMsg.num_scanline
		msg.min_range = objMsg.min_range
		msg.max_range = objMsg.max_range
		msg.range = ranges
		msg.points = points
		msg.splitting_index = objMsg.splitting_index
		msg.header = objMsg.header
		msg.jointState = js
		# objMsg.points = points

		## FOR VISUALIZATION
		self.points3D = [ p[1] for p in points3D if p[0] is not None ]

		return Empty(), msg

	def loop( self ):

		width = 700

		field = np.zeros( (width, width, 3), dtype = np.uint8 )
		field[:,:,1] = 255

		for ii in range( 0, width, 100 ):
			cv2.line( field, (ii, 0), (ii, width), (0,0,0), 1 )
			cv2.line( field, (0, ii), (width, ii), (0,0,0), 1 )

		pt1 = ( width / 2, 550 )
		pt2 = ( (width / 2)-15, width )
		pt3 = ( (width / 2)+15, width )

		triangle_cnt = np.array( [pt1, pt2, pt3] )

		# cv2.drawContours(field, [triangle_cnt], 0, (0, 0, 255), -1)
		# print self.points3D
		for x, y, z in self.points3D:

			x = int( x * 100.0 )
			y = int( y * 100.0 )
			y = (width / 2) - y
			x = width - x

			cv2.circle( field, (y,x), 3, (0,0,255), -1 )

		cv2.imshow( "img", field )
		cv2.waitKey( 1 )

	def end( self ):
		cv2.destroyAllWindows( )			

vision_module = Vision( )
kinematic_module = Kinematic( )