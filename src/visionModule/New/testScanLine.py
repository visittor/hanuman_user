from visionManager.visionModule import KinematicModule, VisionModule

from std_msgs.msg import Empty

from newbie_hanuman.msg import scanlinePointClound, localizationMsg
from newbie_hanuman.msg import point2D

from utility.HanumanForwardKinematic import *

from scanLine2 import findBoundary, findChangeOfColor, findLinearEqOfFieldBoundary, findNewLineFromRansac
from imageProcessingModule.hog_svm import HOG_SVM, HOG_MLP

from skimage import measure, feature
from scipy.spatial.distance import cdist

import numpy as np 
import cv2

import rospy

import os

DIMENSION_FOR_SIMULATION = [ 0.49088, 0.0, 0.0205, 0.01075, 0.046, 0.034, -0.002, 0.0 ]
DIMENSION_FOR_REAL_WORLD = [ 0.42889, 0.0055, 0.00187, 0.0254, 0.05, 0.0, 0.0, 14.34]

# FX = 381.36246688113556
# FY = 381.36246688113556
FX = 577.55352783
FY = 580.48583984

FootballModelPath =  os.path.join( os.getenv( 'ROS_WS' ), "src/hanuman_user/config/model/real_model_with_prob.pkl" )
GoalModelPath = os.path.join( os.getenv( 'ROS_WS' ), "src/hanuman_user/config/model/model_goal_gray.pkl" ) 


# setNewRobotConfiguration( *DIMENSION_FOR_SIMULATION )

class Vision( VisionModule ):

	def __init__( self ):
		super( Vision, self ).__init__( )
		self.objectsMsgType = localizationMsg

		self.predictor = HOG_SVM( FootballModelPath, GoalModelPath, 0.80, rectangleThreshold=30 )

	def _setColorConfig( self, colorConfig ):

		super( Vision, self )._setColorConfig( colorConfig )

		self.greenID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'green' ][ 0 ]
		self.whiteID = [ colorDict.ID for colorDict in self.colorConfig.colorDefList if colorDict.Name == 'white' ][ 0 ]

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
		kernel = np.ones( (20,3), dtype=np.uint8 )
		kernel[:,0] = 0
		kernel[:,2] = 0
		whiteObject_noline = cv2.morphologyEx( whiteObjectInFieldMask, cv2.MORPH_OPEN, kernel )
		
		whiteObjectContours_noline = cv2.findContours( whiteObject_noline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

		whiteObjectContours.extend( whiteObjectContours_noline )

		cv2.imshow( 'gg', whiteObject_noline )
		cv2.waitKey(1)

		return whiteObjectContours_noline

	def ImageProcessingFunction( self, img, header ):

		stepSize = 20 

		imgH, imgW = img.shape[:2]

		color_map = img[:, :, 1].copy()

		fieldContour, fieldMask = findBoundary( color_map, 2 )
		
		boundaryLine = findLinearEqOfFieldBoundary( fieldContour )
		
		ransacContours =  [ [0, imgH] ]
		for m, c, x0, xf in boundaryLine:
			ransacContours.append( [ x0, m*x0 + c ] )
			ransacContours.append( [ xf, m*xf + c ] )
		ransacContours.append( [imgW, imgH] )
		ransacContours = np.vstack( ransacContours ).astype(int).reshape(-1,1,2)

		pointClound = findChangeOfColor( color_map, self.whiteID, self.greenID, mask=fieldMask, step = stepSize )

		points = []
		splitIndexes = []
		
		for i,scanline in enumerate(pointClound):

			for x,y in scanline:
				points.append( point2D( x = x, y = y ) )

		splitIndexes.append( len( points ) )

		for p in fieldContour[1:-1:stepSize,0,:]:

			if p[1] < 10:
				continue
			points.append( point2D( x = p[0], y = p[1] ) )

		newFieldContour = findNewLineFromRansac( fieldContour, 640, 480 )
		# goalList = findGoal( newFieldContour, color_map, goalColorID = 2 )
		goalList = []

		landmarkName = []
		landmarkPose = []
		landmarkConfidence = []

		if len(boundaryLine) == 2:
			intersect_x = boundaryLine[0][3]
			intersect_y = boundaryLine[0][0]*intersect_x + boundaryLine[0][1]

			landmarkName.append( 'field_corner' )
			landmarkPose.append( point2D( x = intersect_x, y = intersect_y ) )
			landmarkConfidence.append( 0.85 )

		self.whiteObjectContours = self.getWhiteObjectContour( color_map, ransacContours )

		canExtract = self.predictor.extractFeature( img[:,:,0], self.whiteObjectContours, 
									objectPointLocation="bottom" )

		if canExtract:

			self.predictor.predict()

			goalList = self.predictor.getGoal()
			foundBall = self.predictor.getBestRegion()

			if foundBall:

				landmarkName.append( 'ball' )

				#	get bounding box object not only position
				bestBounding = self.predictor.getBestBoundingBox()
				#	get another point of object
				botX, botY = bestBounding.bottom

				landmarkPose.append( point2D( botX,
											   botY ) )

				landmarkConfidence.append( bestBounding.footballProbabilityScore )

			for goalObj in goalList:

				landmarkName.append( 'goal' )
				botX, botY = goalObj.bottom
				landmarkPose.append( point2D( botX,
										   botY ) )

				landmarkConfidence.append( goalObj.goalProbabilityScore )

		msg = localizationMsg( )
		msg.pointClound.num_scanline = 8
		msg.pointClound.min_range = 0
		msg.pointClound.max_range = 2**16 - 1
		msg.pointClound.range = []
		msg.pointClound.points = points
		msg.pointClound.splitting_index = splitIndexes
		msg.header = header

		msg.landmark.names = landmarkName
		msg.landmark.pose = landmarkPose
		msg.landmark.confidences = landmarkConfidence

		return msg

	def visualizeFunction( self, img, msg ):

		super( Vision, self ).visualizeFunction( img, msg )

		cv2.drawContours( img, self.whiteObjectContours, -1, (0,0,255), 1 )

		for point in msg.pointClound.points:
			x = int( point.x )
			y = int( point.y )
			# cv2.circle(img,(x,y), 4, (0,0,0), -1)
			# cv2.circle(img,(x,y), 3, (0,0,255), -1)
			img[ y, x ] = [0,0,0]

		for n, p in zip( msg.landmark.names, msg.landmark.pose ):

			x, y = int(p.x), int(p.y)

			if n == 'goal':
				color = (0, 255, 255)

			elif n == 'field_corner':
				color = (255, 0, 255)

			else:
				color = (0,0,0)

			cv2.circle( img, (x,y), 2, (0,0,0), -1 )
			cv2.circle( img, (x,y), 4, color, -1 )

class Kinematic( KinematicModule ):

	def __init__( self ):
		super( Kinematic, self ).__init__(  )

		robotConfigPathStr = rospy.get_param( '/robot_config', None )
		config = configobj.ConfigObj( robotConfigPathStr )
		loadDimensionFromConfig( robotConfigPathStr )
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

		self.objectsMsgType = localizationMsg
		self.posDictMsgType = Empty

		self.points3D = []
		self.points2D = []
		self.landmarkName = []
		self.landmarkPose3D = []

		self.circle = None

		self.subscribeMotorCortex = True

	def findCircle_ransac( self, point3D ):

		def is_data_valid( random_data ):

			distMat = cdist( random_data, random_data )
			distAvg = (np.sum( distMat )/2) / len( random_data )

			if distAvg < 0.2:
				return False

			return True

		def is_model_valid(model, *random_data):
			x, y, r = model.params

			if r > 0.9:
				return False

			return True
		if len( point3D ) < 3:
			return
		model, inliers = measure.ransac(point3D, measure.CircleModel,
                                min_samples=3, residual_threshold=0.05,
                                max_trials=500, is_model_valid = is_model_valid,
                                is_data_valid = is_data_valid)

		if model is None:
			return

		x, y, r = model.params

		return x, y, r if np.sum(inliers) > 15 and r is not None else None

	def getDensityProbability( self, x, mean, sigma ):

		prob = math.exp( -((x-mean)**2)/(2*sigma**2) ) / (math.sqrt(2*math.pi)*sigma)

		return prob

	def getDensityProbability_normalize( self, x, mean, sigma ):
		prob = self.getDensityProbability( x, mean, sigma )
		prob_perfect = self.getDensityProbability( mean, mean, sigma )

		return prob / prob_perfect

	def kinematicCalculation( self, objMsg, js, cortexMsg, rconfig=None ):
		# print getRobotConfiguration()
		self.points2D = [ [p.x, p.y] for p in objMsg.pointClound.points ]

		pitch = math.radians(cortexMsg.pitch) if cortexMsg is not None else 0.0
		roll = math.radians(cortexMsg.roll)	if cortexMsg is not None else 0.0
		# print math.degrees(pitch), math.degrees(roll)
		roll = pitch = 0.0
		tranvec = np.zeros( (3,1) )
		rotvec = np.array( [roll, pitch, 0.0] )

		HRotate = self.create_transformationMatrix(tranvec, rotvec, 
													'rpy', order="tran-first")
		
		H = getMatrixForForwardKinematic( js.position[0], js.position[1], roll, pitch )
		H = np.matmul( HRotate, H )

		points3D = self.calculate3DCoor( self.points2D, HCamera = H )

		ranges = []
		points = []
		for plane, p3D in points3D:
			if plane is None:
				ranges.append( -1 )

			else:
				x, y = p3D[:2]
				x *= 1.3
				y *= 1.3
				ranges.append( np.sqrt( x**2 + y**2 ) )
				points.append( point2D( x = x, y = y ) )

		self.circle = self.findCircle_ransac( np.array([[p.x,p.y] for p in points]) )
	
		point3D_landmark = self.calculate3DCoor( [ [p.x,p.y] for p in objMsg.landmark.pose ], HCamera = H )

		landmarkName = []
		landmarkPose3D = []
		landmarkConfidence = []
		for i, (plane, p3D) in enumerate( point3D_landmark ):
			if plane is None:
				continue

			x, y = p3D[:2]
			landmarkName.append( objMsg.landmark.names[i] )
			landmarkPose3D.append( point2D( x = x, y = y ) )
			landmarkConfidence.append( objMsg.landmark.confidences[i] )

		if self.circle is not None and self.circle[0] is not None and self.circle[1] is not None and self.circle[2] is not None:
			landmarkName.append( 'circle' )
			landmarkPose3D.append( point2D( x = self.circle[0], y = self.circle[1] ) )
			landmarkConfidence.append( self.getDensityProbability_normalize( self.circle[2], 
																			0.75, 0.5 ) )

		pointCloundMSG = scanlinePointClound( )
		pointCloundMSG.num_scanline = objMsg.pointClound.num_scanline
		pointCloundMSG.min_range = objMsg.pointClound.min_range
		pointCloundMSG.max_range = objMsg.pointClound.max_range
		pointCloundMSG.range = ranges
		pointCloundMSG.points = points
		pointCloundMSG.splitting_index = objMsg.pointClound.splitting_index
		pointCloundMSG.header = objMsg.header
		pointCloundMSG.jointState = js

		msg = localizationMsg( )
		msg.header = objMsg.header
		msg.pointClound = pointCloundMSG

		msg.landmark.names = landmarkName
		msg.landmark.pose = landmarkPose3D
		msg.landmark.confidences = landmarkConfidence

		# objMsg.points = points
		## FOR VISUALIZATION
		self.points3D = [ p[1] for p in points3D if p[0] is not None ]
		self.landmarkName = landmarkName
		self.landmarkPose3D = landmarkPose3D

		return Empty(), msg

	def loop( self ):

		width = 700
		pixPerMetre = 50.0

		field = np.zeros( (width, width, 3), dtype = np.uint8 )
		field[:,:,1] = 255

		for ii in range( 0, width, int( pixPerMetre )  ):
			cv2.line( field, (ii, 0), (ii, width), (0,0,0), 1 )
			cv2.line( field, (0, ii), (width, ii), (0,0,0), 1 )

		pt1 = ( width / 2, 550 )
		pt2 = ( (width / 2)-15, width )
		pt3 = ( (width / 2)+15, width )

		triangle_cnt = np.array( [pt1, pt2, pt3] )

		# cv2.drawContours(field, [triangle_cnt], 0, (0, 0, 255), -1)
		# print self.points3D
		for x, y, z in self.points3D:

			x = int( x * pixPerMetre )
			y = int( y * pixPerMetre )
			y = (width / 2) - y
			x = width - x

			cv2.circle( field, (y,x), 3, (0,0,255), -1 )

		blank = np.zeros( (480,640,3), dtype = np.uint8 )

		for x,y in self.points2D:
			x = int(x)
			y = int(y)
			cv2.circle( blank, (x,y), 3, (0,0,255), -1 )

		for p, n in zip( self.landmarkPose3D, self.landmarkName ):

			x, y = int( p.x * pixPerMetre ), int( p.y * pixPerMetre )
			y = (width / 2) - y
			x = width - x

			if n == 'field_corner':
				color = (255, 0, 255)
			elif n == 'goal':
				color = (0, 255, 255)
			elif n == 'circle':
				color = (255,255,0)
			else:
				color = (0,0,0)

			cv2.circle( field, (y,x), 10, color, -1 )

		if self.circle is not None:
			x, y, r = self.circle

			if x is None or y is None or r is None:
				pass

			else:

				x = int(x*pixPerMetre)
				y = int(y*pixPerMetre)
				r = int(r*pixPerMetre)
				y = (width/2) - y
				x = width - x

				cv2.circle( field, (y,x), r, (255,0,0), 3 )

		cv2.imshow( "3D", field )
		# cv2.imshow( "2D", blank)
		k = cv2.waitKey( 1 )

		if k == ord('s') and len( self.points3D ) > 0:
			np.save( '/tmp/scanline.npy', self.points3D )
			print 'save to /tmp/scanline.npy'

	def end( self ):
		cv2.destroyAllWindows( )			

vision_module = Vision( )
kinematic_module = Kinematic( )