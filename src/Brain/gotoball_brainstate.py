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

from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

from newbie_hanuman.msg import postDictMsg

import numpy as np

import time
import rospy

########################################################
#
#	GLOBALS
#

DefaultDistanceToKickingState = 0.25 #m
DefaultAngleToTrackingState = 15 #degree
DefaultObject = 'ball'

ErrorFootballAndCenterDistance = 0.6 #0-1 floating point distance 

DefaultLoopTimeToLookAtObject = 1.0

########################################################
#
#	EXCEPTION DEFINITIONS
#

########################################################
#
#	HELPER FUNCTIONS
#

########################################################
#
#	CLASS DEFINITIONS
#

class FollowBall( FSMBrainState ):

	def __init__( self, kickingState = None, trackingState = None ):
		super( FollowBall, self ).__init__( 'FollowBall' )

		#	Set previous state
		self.trackingState = trackingState

		#	Set next state
		self.kickingState = kickingState
		
		#	initial object
		self.objectIndex = None
		
		#	time stamp
		self.stampTime = 0
		
	def firstStep( self ):

		rospy.loginfo( "Enter follow ball state" )

		self.objectIndex = self.rosInterface.visionManager.object_name.index( DefaultObject )

		if self.rosInterface.visionManager.object_confidence[ self.objectIndex ] < 0.5:
			self.SignalChangeSubBrain( self.trackingState )
			
		self.stampTime = time.time()
			
		
	def step( self ):

		#	initial current step time
		currentStepTime = time.time()

		#	get position from y-axis
		yMagnitude = self.rosInterface.visionManager.pos3D_cart[ self.objectIndex ].y
		xMagnitude = self.rosInterface.visionManager.pos3D_cart[ self.objectIndex ].x
		
		#	get theta of ball wrt robot
		thetaBallWrtRobot = self.rosInterface.visionManager.pos2D_polar[ self.objectIndex ].y
		
		
		rospy.logdebug( "X distance : {}".format( xMagnitude ) )
		rospy.logdebug( "Y distance : {}".format( yMagnitude ) )
		rospy.logdebug( "theta of robot wrt : {}".format( thetaBallWrtRobot ) )
		
		
		#	change state when robot is outside theta threshold
		if abs( thetaBallWrtRobot ) >= np.deg2rad( DefaultAngleToTrackingState ):
			self.SignalChangeSubBrain( self.trackingState )
		
		#	go forward to football
		self.rosInterface.LocoCommand(	velX = 0.2,
						velY = 0,
						omgZ = 0,
						commandType = 0,
						ignorable = False )
		
		#	get error x and error y
		errorX = self.rosInterface.visionManager.object_error[ self.objectIndex ].x
		errorY = self.rosInterface.visionManager.object_error[ self.objectIndex ].y
		
		#	calculate error from center camera#
#		errorEuclidianDistanceBallOnImage = np.sqrt( np.power( errorX, 2 ) + np.power( errorY, 2 ) )
#		
#		rospy.logdebug( "error from center of the camera : {}".format( errorEuclidianDistanceBallOnImage ) )
		
#		if errorEuclidianDistanceBallOnImage  > ErrorFootballAndCenterDistance:
		timeRemain = currentStepTime - self.stampTime
		if timeRemain >= DefaultLoopTimeToLookAtObject:
			self.rosInterface.Pantilt( command = 2, pattern = DefaultObject )
			self.stampTime = time.time()
			
		#	arrival at desire position			
		if xMagnitude <= DefaultDistanceToKickingState:
		
			#	STOPPP
			self.rosInterface.LocoCommand(	velX = 0,
							velY = 0,
							omgZ = 0,
							commandType = 0,
							ignorable = False )
			
			self.SignalChangeSubBrain( self.kickingState )				
		
							
							
		

