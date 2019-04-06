#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun (NeverHoliday) Hayeeyama
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

from scanLine2 import findBoundary, findLinearEqOfFieldBoundary, findNewLineFromRansac

from imageProcessingModule.hog_svm import HOG_SVM
from imageProcessingModule import findGoal

import rospy

import numpy as np
import cv2

import configobj

import time

from scipy import spatial

########################################################
#
#	GLOBALS
#

#ModelPath = os.path.join( os.getenv( 'ROS_WS' ), 'src/hanuman_user/config/model/real_model_with_prob.pkl' )
ModelPath = os.path.join( os.getenv( 'ROS_WS' ), 'src/hanuman_user/config/model/model_gray.pkl' )

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
	'''	ImageProcessingClass
	
	Do autosegment
	
	'''
	
	def __init__( self ):
		
		super( ImageProcessing, self ).__init__()
		
		#	define type
		self.objectsMsgType = visionMsg
		
		#	contour visualization
		self.contourFieldVis = None
		self.ransacContourVis = None
		
		#	get model object
		#	positive threshold is 0.70
		self.predictor = HOG_SVM( ModelPath, 0.70 )
		
	def calculateError( self, imageWidth, imageHeight, centerX, centerY ):

		errorX = ( centerX - imageWidth / 2. ) / ( imageWidth / 2. )
		errorY = ( centerY - imageHeight / 2. ) / ( imageHeight / 2. )

		return errorX, errorY
	
# 	def findGoal( self, ransacContours, marker ):
		
# 		#	
# 		#	two line scan
# 		#
# 		ransacContours1 = ransacContours.copy()
# 		ransacContours2 = ransacContours.copy()
		
# 		#	offset
# 		ransacContours1[ :, :, 1 ] = ransacContours1[ :, :, 1 ] + 10
# 		ransacContours2[ :, :, 1 ] = ransacContours2[ :, :, 1 ] + 15
		
# 		#	initial previous point
# 		previousPoint1 = ransacContours1[ 1 ]
# 		previousPoint2 = ransacContours2[ 1 ]
		
# 		#	initial change list
# 		changeList1 = list()
# 		changeList2 = list()
		
# #		print "	ransac 1 "
# 		#	scan two lines
# 		for point in ransacContours1[ 1:-1 ]:
			
# 			x = point[ 0 ][ 0 ]
# 			y = point[ 0 ][ 1 ]
			
# 			currentMarker = marker[ y, x ]
# 			previousMarker = marker[ previousPoint1[ 0 ][ 1 ], previousPoint1[ 0 ][ 0 ] ]
			
# 			if currentMarker != previousMarker:
# #				print "		current marker : {}".format( currentMarker )
# #				print "		previous marker : {}".format( previousMarker )	
				
# 				if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 2:
# 					#isChange = True
# 					changeList1.append( point )
			
# 			previousPoint1 = point
		
# #		print "	ransac 2 "
		
# 		#	scan two lines
# 		for point in ransacContours2[ 1:-1 ]:
		
# 			x = point[ 0 ][ 0 ]
# 			y = point[ 0 ][ 1 ]
			
# 			currentMarker = marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ]
# 			previousMarker = marker[ previousPoint2[ 0 ][ 1 ], previousPoint2[ 0 ][ 0 ] ]
			
# 			if currentMarker != previousMarker:
# #				print "		current marker : {}".format( currentMarker )
# #				print "		previous marker : {}".format( previousMarker )
				
# 				if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 2:
# 					#isChange = True
# 					changeList2.append( point )
			
# 			previousPoint2 = point
		
# 		#	change list to array for find distance
# 		changeArray1 = np.array( changeList1 ).reshape( -1, 2 )
# 		changeArray2 = np.array( changeList2 ).reshape( -1, 2 )
# 		#print changeList2
# 		#	find distance for grouping point
# 		if len( changeArray1 ) != 0 and len( changeArray2 ) != 0:
		
# 			distanceMatrix = spatial.distance.cdist( changeArray1, changeArray2 ) 
# 			minIdxAxis_0 = np.argmin( distanceMatrix, axis = 0 )
# 			minIdxAxis_1 = np.argmin( distanceMatrix, axis = 1 )
	
# 			goalPointList = list()
			
# 			#print minIdxAxis_0, minIdxAxis_1
			
# 			for i, j in enumerate( minIdxAxis_1 ):
# 				if distanceMatrix[ i, j ] < 5.5:
# 					x1, y1 = changeList1[ i ][ 0 ][ 0 ], changeList1[ i ][ 0 ][ 1 ]
# 					x2, y2 = changeList2[ j ][ 0 ][ 0 ], changeList2[ j ][ 0 ][ 1 ]

# 					goalPointList.append( ( x1, y1 ) )
					
# 			return goalPointList
			
# #			if len( goalPointList ) == 4:
# #								
# #				goalLeft = goalPointList[ 0 : 2 ]
# #				goalRight = goalPointList[ 2 : 4 ]
# #				
# #				return ( goalLeft, goalRight )
# #				
# #			else:
# #				return ( None, None )
# 		#
# 		else:
# 			return [ None ]	
				
	def ImageProcessingFunction( self, img, header ):
		'''	ImageProcessingFunction
		'''
		
		#	get image property
		imageHeight = img.shape[ 0 ]
		imageWidth = img.shape[ 1 ]
		
		#	get gray scale image
		imageGray = img[ :, :, 0 ].copy()
		
		#	initial ball position
		bestPosition = Point32()
		ballErrorList = Point32()
		ballConfidence = 0.0
	
		#	get marker (on channel 1)
		marker = img[ :, :, 1 ]  
		
		#	get field boundary
		fieldContour, fieldMask = findBoundary( marker, 2, flip = False )
		self.contourFieldVis = fieldContour
		
		#
		#	do ransac
		#
		
		coeffList = findLinearEqOfFieldBoundary( fieldContour[ 1:-1 ].copy() )
		
		
		if len( coeffList ) > 1:
			#	find y intersect
			xIntersect = coeffList[ 0 ][ 3 ]
			m = coeffList[ 0 ][ 0 ]
			c = coeffList[ 0 ][ 1 ]

			yIntersect = ( m * xIntersect ) + c
			
			intersectPoint = Point32( x = xIntersect, y = yIntersect, z = 0.0 )
			intersectPointConfidence = 1.0
			
		else:
			intersectPoint = Point32()
			intersectPointConfidence = 0.0
			
		ransacContour = findNewLineFromRansac( fieldContour, imageWidth, imageHeight )
		
#		#	cut first and last point
#		fieldContourMiddlePoints = fieldContour[ 1:-1 ].copy()
#		
#		#	get ransac result
#		linearCoefficiant = findLinearEqOfFieldBoundary( fieldContourMiddlePoints )  
#		
#		#	create list
#		pointList = list()
#		
#		for lineCoeff in linearCoefficiant:
#			
#			#	get linear coeff
#			x0 = lineCoeff[ 2 ]
#			xf = lineCoeff[ 3 ]
#			m = lineCoeff[ 0 ]
#			c = lineCoeff[ 1 ]
#			
#			#	calculate y from x
#			x = np.arange( x0, xf, dtype = np.int64 )
#			y = np.int64( m * x ) + int( c ) 
#			
#			contour = np.vstack( ( x, y ) ).transpose()
#			contour = contour.reshape( -1, 1, 2 )
#			
#			pointList.append( contour )
#		
#		if len( pointList ) > 1:
#			ransacContour = np.vstack( pointList )
##			print pointList[ 0 ].shape
##			print pointList[ 1 ].shape
##			print contour.shape
#		else:
#			ransacContour = pointList[ 0 ]
#			
#		#	concat first and last point
#		firstPoint = np.array( [ [ [ 0, img.shape[ 0 ] - 1 ] ] ] )
#		lastPoint = np.array( [ [ [ img.shape[ 1 ] - 1, img.shape[ 0 ] - 1 ] ] ] )
#		ransacContour = np.concatenate( ( firstPoint, ransacContour, lastPoint ) )	
		self.ransacContourVis = ransacContour
		
		#	get new field boundaty from ransac technic
		newFieldMask = np.zeros( marker.shape, dtype = np.uint8 )
		cv2.drawContours( newFieldMask, [ ransacContour ], 0, 1, -1 )
		 
		#
		#	get white object and predict ball
		#
		whiteObject = np.zeros( marker.shape, dtype = np.uint8 )
		whiteObject[ marker == 5 ] = 1
		
		#	get white object only the field
		whiteObjectInField = whiteObject * newFieldMask
		whiteObjectInField *= 255

		#	find contour from white object in field
		whiteContours = cv2.findContours( whiteObjectInField, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]
		
		#
		#	detect goal
		#
		
		goal = findGoal( ransacContour, marker )
#		goalLeft, goalRight = None, None
#		
#		if goalLeft is not None and goalRight is not None:
#			
#			#	find middle point of goalpost
#			goalLeftPoint = ( ( ( goalLeft[ 0 ][ 0 ] + goalLeft[ 1 ][ 0 ] ) / 2 ), ( ( goalLeft[ 0 ][ 1 ] + goalLeft[ 1 ][ 1 ] ) / 2 ) )
#			goalRightPoint = ( ( ( goalRight[ 0 ][ 0 ] + goalRight[ 1 ][ 0 ] ) / 2 ), ( ( goalRight[ 0 ][ 1 ] + goalRight[ 1 ][ 1 ] ) / 2 ) )
#			
#			goalLeftPoint = Point32( goalLeftPoint[ 0 ], goalRightPoint[ 1 ], 0.0 )
#			goalRightPoint = Point32( goalRightPoint[ 0 ], goalRightPoint[ 1 ], 0.0 )
#			
#			goalLeftConfidence = 1.0
#			goalRightConfidence = 1.0
#		else:
#			goalLeftPoint = Point32()
#			goalRightPoint = Point32()
#			
#			goalLeftConfidence = 0.0
#			goalRightConfidence = 0.0
			
			
		#
		#	detect goal
		#
				
		#	extract feature and predict it
		extractStatus = self.predictor.extractFeature( imageGray, whiteContours, objectPointLocation = 'bottom' )
		
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
		
		 
		#
		#	create msg
		#
		msg = visionMsg()
		
		#	assign value to message
		msg.object_name = [ 'ball', ]
		msg.pos2D = [ bestPosition, ]
		msg.imgH = imageHeight
		msg.imgW = imageWidth
		msg.object_error = [ ballErrorList ]
		msg.object_confidence = [ ballConfidence ]
		msg.header.stamp = rospy.Time.now()
		
		for idx, g in enumerate( goal ):
			if g is None:
				continue
			msg.object_name.append( 'goal_{}'.format( idx ) )
			x, y = g[ 0 ], g[ 1 ]
			x -= 3
			while y < 480 and marker[ y, x ] != 2:
				y += 1
				
			
			msg.pos2D.append( Point32( x = float( x ), y = float( y ), z = 0 ) )
			
			#	calculate error
			errorX, errorY = self.calculateError( imageWidth, imageHeight, g[ 0 ], g[ 1 ] )
			msg.object_error.append( Point32( x = errorX, y = errorY, z = 0 ) )
			
			msg.object_confidence.append( 1.0 )
			
		#	append intersect point
		msg.object_name.append( 'intersect_point' )
		errorX, errorY = self.calculateError( imageWidth, imageHeight, intersectPoint.x, intersectPoint.y )
		msg.object_error.append( Point32( x = errorX, y = errorY, z = 0.0 ) )
		msg.pos2D.append( intersectPoint )
		msg.object_confidence.append( intersectPointConfidence )
			
		return msg
		
		
	def visualizeFunction(self, img, msg):
		
		#	visualize contour naja
		#cv2.drawContours( img, [ self.contourFieldVis ], 0, (   0, 0, 255 ), 1 )
		cv2.drawContours( img, [ self.ransacContourVis ], 0, ( 255, 0, 0 ), 2 )
		#for colorDict in self.colorConfig.colorDefList:
		#img[ img[:,:,1] == self.colorConfig.colorDefList[ 1 ].ID ] = eval(self.colorConfig.colorDefList[ 1 ].RenderColor_RGB)
		
		for colorDict in self.colorConfig.colorDefList:
			img[ img[:,:,1] == colorDict.ID ] = eval(colorDict.RenderColor_RGB)
		
		for n, p in zip( msg.object_name, msg.pos2D ):
			
			if n.split('_')[0] == 'goal':
				cv2.circle( img, ( int(p.x), int(p.y) ), 10, ( 0, 0, 255 ), -1 )
		
		if msg.object_confidence[ 0 ] > 0.9:
			cv2.circle( img, ( msg.pos2D[ 0 ].x, msg.pos2D[ 0 ].y ), 10, ( 255, 0, 0 ), -1 )
		
		if msg.object_confidence[ -1 ] > 0.9:	
			cv2.circle( img, ( int( msg.pos2D[ -1 ].x ), int( msg.pos2D[ -1 ].y ) ), 10, ( 0, 128, 128 ), -1 )
		
#		if msg.object_confidence[ 1 ] > 0.9:
#			cv2.circle( img, ( msg.pos2D[ 1 ].x, msg.pos2D[ 1 ].y ), 3, ( 0, 0, 255 ), -1 )
#		
#		if msg.object_confidence[ 2 ] > 0.9:
#			cv2.circle( img, ( msg.pos2D[ 2 ].x, msg.pos2D[ 2 ].y ), 3, ( 0, 0, 255 ), -1 )
		
		#print self.colorConfig.colorDefList[ 1 ]
