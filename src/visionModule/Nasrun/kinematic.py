import sys
import os

import cv2
import numpy as np

from skimage import measure, feature
from scipy.spatial.distance import cdist

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

from newbie_hanuman.msg import scanlinePointClound, localizationMsg
from newbie_hanuman.msg import point2D

from configobj import ConfigObj

from visionManager.visionModule import VisionModule, KinematicModule
from utility.HanumanForwardKinematic import loadDimensionFromConfig, getMatrixForForwardKinematic
from utility.transformationModule import Project3Dto2D, Project2Dto3D, getInverseHomoMat

from newbie_hanuman.msg import visionMsg, postDictMsg

from geometry_msgs.msg import Point32 

import math

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

			elif r < 0.5:
				return False

			return True
		if len( point3D ) < 3:
			return
		model, inliers = measure.ransac(point3D, measure.CircleModel,
                                min_samples=3, residual_threshold=0.05,
                                max_trials=200, is_model_valid = is_model_valid,
                                is_data_valid = is_data_valid)

		if model is None:
			return

		x, y, r = model.params

		thr = max(0.1 * len( point3D ), 20)

		return x, y, r if np.sum(inliers) > thr and r is not None else None

	def getDensityProbability( self, x, mean, sigma ):

		prob = math.exp( -((x-mean)**2)/(2*sigma**2) ) / (math.sqrt(2*math.pi)*sigma)

		return prob

	def getDensityProbability_normalize( self, x, mean, sigma ):
		prob = self.getDensityProbability( x, mean, sigma )
		prob_perfect = self.getDensityProbability( mean, mean, sigma )

		return prob / prob_perfect

	def kinematicCalculation( self, objMsg, js, cortexMsg, rconfig=None ):
		t0 = time.time( )

		pointsClound2D = [ ]
		boundary2D = [ ]
		object2D = [ ]
		indexList = [ ]

		for i, (p, n) in enumerate(zip( objMsg.pos2D, objMsg.object_name )):
			if n == 'point':
				pointsClound2D.append( [p.x, p.y] )
			elif n == 'field':
				boundary2D.append( [p.x, p.y] )
			else:
				object2D.append( [p.x, p.y] )
				indexList.append( i )
		t1 = time.time()

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

		t2 = time.time()
		pointsClound3D = self.calculate3DCoor( pointsClound2D, HCamera = H )
		t3 = time.time()
		boundary3D = self.calculate3DCoor( boundary2D, HCamera = H )
		t4 = time.time()
		object3D = self.calculate3DCoor( object2D, HCamera = H )
		t5 = time.time()

		polarList = []
		cartList = []
		names = []
		confidences = []
		errorList = []
		pos2DList = []

		landmarkPose3D = []
		ranges = []
		points = []
		splitIndexes = []

		imgH = objMsg.imgH
		imgW = objMsg.imgW

		for (plane, p3D), i in zip(object3D, indexList ):
			name = objMsg.object_name[i]
			confidence = objMsg.object_confidence[i]
			p2D = objMsg.pos2D[i]
			err = objMsg.object_error[i]

			if plane is None:
				pass

			else:
				x, y = p3D[:2]
				
				cartList.append( Point32( x = x, y = y ) )
				landmarkPose3D.append( point2D( x = x, y = y ) )

				rho = math.sqrt( x**2 + y**2 )
				phi = math.atan2( y, x )

				polarList.append( Point32( x = rho, y = phi) )

				names.append( name )
				confidences.append( confidence )
				errorList.append( Point32( x = err.x, y = err.y ) )
				pos2DList.append( p2D )

		t6 = time.time()

		for plane, p3D in pointsClound3D:
			if plane is None:
				continue

			else:
				x, y = p3D[:2]
				# x *= 0.8
				# y *= 0.8
				ranges.append( math.sqrt( x**2 + y**2 ) )
				points.append( point2D( x = x, y = y ) )

		splitIndexes.append( len( points ))

		t7 = time.time()

		for plane, p3D in boundary3D:
			if plane is None:
				continue

			else:
				x, y = p3D[:2]
				# x *= 0.8
				# y *= 0.8
				ranges.append( math.sqrt( x**2 + y**2 ) )
				points.append( point2D( x = x, y = y ) )

		circle = self.findCircle_ransac( np.array([[p.x,p.y] for p in points]) )
		if circle is not None and circle[0] is not None and circle[1] is not None and circle[2] is not None:
			names.append( 'circle' )
			landmarkPose3D.append( point2D( x = circle[0], y = circle[1] ) )
			confidences.append( self.getDensityProbability_normalize( circle[2], 
																	0.75, 0.5 ) )
		t8 = time.time()

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

		pointCloundMSG = scanlinePointClound( )
		pointCloundMSG.num_scanline = 20
		pointCloundMSG.min_range = 0.5
		pointCloundMSG.max_range = 4
		pointCloundMSG.range = ranges
		pointCloundMSG.points = points
		pointCloundMSG.splitting_index = splitIndexes
		pointCloundMSG.jointState = js

		msgLocalize = localizationMsg( )
		msgLocalize.pointClound = pointCloundMSG

		msgLocalize.landmark.names = names
		msgLocalize.landmark.pose = landmarkPose3D
		msgLocalize.landmark.confidences = confidences

		t9 = time.time()

		print "Create npy :", t1 - t0
		print "Creat H mat:", t2 - t1
		print "Cal pointC :", t3 - t2
		print "Cal boundy :", t4 - t3
		print "Cal objMsg :", t5 - t4
		print "Add obj2lis:", t6 - t5
		print "Add poitntC:", t7 - t6
		print "Add boundar:", t8 - t7
		print "create msg :", t9 - t8
		print "Overall	  :", t9 - t0
		print '---------\n'

		return msg, msgLocalize