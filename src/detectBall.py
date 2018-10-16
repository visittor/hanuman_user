#!/usr/bin/env python


from visionManager.visionModule import VisionModule, KinematicModule
from newbie_hanuman.msg import visionMsg, postDictMsg

import rospy

import numpy as np
import cv2

import os
import sys

import time

class ImageProcessing( VisionModule ):

	def __init__( self ):
		super(ImageProcessing, self).__init__()

		#   define message
		self.objectsMsgType = visionMsg

		colorList = [ 0, 142, 207, 59, 255, 255 ]

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
				cx = int( M['m10'] / M['m00'] )
				cy = int( M['m01'] / M['m00'] )
			except ZeroDivisionError:
				cx, cy = self.__previousPosition

			ballPosition = [ cx, cy ]
			isDetectBall = True
			
			#	calculate error
			#	NOTE : edit error y for switch sign for control motor
			errorX = ( ballPosition[ 0 ] - imageWidth / 2. ) / ( imageWidth / 2. )
			errorY = -1 * ( ballPosition[ 1 ] - imageHeight / 2. ) / ( imageHeight / 2. )

		else:
			ballPosition = [ 0, 0 ]
			errorX, errorY = 0., 0.
			isDetectBall = False


		msg = visionMsg()
		msg.ball = ballPosition
		msg.imgH = hsvImage.shape[0]
		msg.imgW = hsvImage.shape[1]
		msg.ball_error = [ errorX, errorY ]
		msg.ball_confidence = isDetectBall
		msg.header.stamp = rospy.Time.now()

		self.__previousPosition = ballPosition

		return msg

	def visualizeFunction(self, img, msg):

		#	draw circle
		if msg.ball_confidence:
			cv2.circle( img, ( msg.ball[ 0 ], msg.ball[ 1 ] ), 10, ( 255, 0, 0 ), -1 )
			print " error X : {}, error Y : {} ".format( msg.ball_error[ 0 ], msg.ball_error[ 1 ] )

class Kinematic( KinematicModule ):
	
	def __init__( self ):
		
		super( Kinematic, self ).__init__()

		#	define object type 
		self.objectsMsgType = visionMsg
		self.posDictMsgType = postDictMsg

	def kinematicCalculation(self, objMsg, joint):
		
		#	get ball error
		errorX, errorY = objMsg.error

		#	publist positon dict message
		msg = postDictMsg()
		msg.ball_cart = [ 1 ] * 2 
		msg.ball_polar = [ 1 ] * 2 
		msg.ball_img = objMsg.ball
		msg.imgW = objMsg.imgW
		msg.imgH = objMsg.imgH
		msg.ball_error = [ errorX, errorY ]
		msg.ball_confidence = objMsg.ball_confidence
		msg.header.stamp = rospy.Time.now()

#	create instance
vision_module = ImageProcessing()
kinematic_module = Kinematic()