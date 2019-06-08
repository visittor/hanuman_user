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

from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

from newbie_hanuman.msg import postDictMsg

import numpy as np
import math

import time
import rospy

from default_config import getParameters

########################################################
#
#	GLOBALS
#

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

class RotateToTheBall( FSMBrainState ):
	
	def __init__( self, failState = "None", successState = "None", lostBallState = "None" ):
		
		super( RotateToTheBall, self ).__init__( "RotateToTheBall" )
		
		self.successState = successState
		self.failState = failState

		self.lostBallState = lostBallState

		self.numFrameNotDetectBall = None

		self.omegaZ = None
		self.smallTheta = None

		self.fx = None

	def initialize( self ):

		#	Get omega_z from config
		self.omegaZ = float( getParameters(self.config, 'VelocityParameter', 'OmegaZWhenRotateToBall'))

		#	Get theta to change state
		self.smallTheta = float( getParameters(self.config, 'ChangeStateParameter', 'SmallDegreeToAlignTheBall'))

		#	Get tilt limit
		self.panLimit = float( getParameters(self.config, 'ChangeStateParameter', 'LimitPanAngleDegree'))

		self.confidenceThr = float( getParameters(self.config, 'ChangeStateParameter', 'BallConfidenceThreshold'))

		#	Get fx and fy from robot config
		self.fx = float( self.config[ "CameraParameters" ][ "fx" ] )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		
		#	re-initial numframe to detect ball
		self.numFrameNotDetectBall = 0
			
	def step( self ):
		
		#	Get vision msg
		visionMsg = self.rosInterface.visionManager

		localPosDict = self.rosInterface.local_map( reset = False ).postDict

		ballErrorY = 0

		#
		# This is logic for changing state.
		#

		#	If detect
		if 'ball' in visionMsg.object_name:
		
			#	when vision manager found the ball
			self.numFrameNotDetectBall = 0

			idxBallVisionObj = visionMsg.object_name.index( 'ball' )
			ballErrorX = visionMsg.object_error[ idxBallVisionObj ].x

			imgW = visionMsg.imgW
			fovWidth = 2 * np.arctan( 0.5 * imgW / self.fx )

			panAngle = ballErrorX * fovWidth / 2

			currentPanAngle = self.rosInterface.pantiltJS.position[ 0 ] + panAngle

			# if math.fabs(currentPanAngle) < math.fabs( math.radians( self.panLimit ) ):
				
			# 	self.SignalChangeSubBrain( self.successState )
				
		if 'ball' in localPosDict.object_name:	

			idxBallLocalObj = localPosDict.object_name.index( 'ball' ) 
			localDistanceX = localPosDict.pos3D_cart[ idxBallLocalObj ].x
			localDistanceY = localPosDict.pos3D_cart[ idxBallLocalObj ].y
			thetaWrtRobotRad = localPosDict.pos2D_polar[ idxBallLocalObj ].y

			if localPosDict.object_confidence[ idxBallLocalObj ] < self.confidenceThr:
				#	it should switch to first state to find the ball
				self.SignalChangeSubBrain( self.lostBallState )

## NOTE : This case is not pratical since localMap not update object using visionMsg while robot is walking.			
			if math.fabs( thetaWrtRobotRad ) <= math.radians( self.smallTheta ):
				#	Back to previous state
				self.SignalChangeSubBrain( self.successState )

		else:
			self.SignalChangeSubBrain( self.lostBallState )

		#
		#	Rotate to ball
		#
		
		#	Get sign to rotate
		direction = 1 if thetaWrtRobotRad > 0 else -1
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = direction * self.omegaZ,
										commandType = 0,
										ignorable = False )

	def leaveStateCallBack( self ):

		self.stopRobotBehavior( )
		
	def stopRobotBehavior( self ):
		#	stop
		# self.rosInterface.LocoCommand( velX = 0.0,
		# 			       			   velY = 0.0,
		# 			       			   omgZ = 0.0,
		# 			       			   commandType = 0,
		# 			       			   ignorable = False )		
		self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   ignorable =  False )

class RotateToTheBall2( FSMBrainState ):
	
	def __init__( self, failState = "None", successState = "None", lostBallState = "None" ):
		
		super( RotateToTheBall2, self ).__init__( "RotateToTheBall2" )
		
		self.successState = successState
		self.failState = failState

		self.lostBallState = lostBallState

		self.numFrameNotDetectBall = None

		self.omegaZ = None
		self.smallTheta = None

		self.timeStart = time.time()

		self.fx = None

	def initialize( self ):

		#	Get omega_z from config
		self.omegaZ = float( getParameters(self.config, 'VelocityParameter', 'OmegaZWhenRotateToBall'))

		#	Get theta to change state
		self.smallTheta = float( getParameters(self.config, 'ChangeStateParameter', 'SmallDegreeToAlignTheBall'))

		#	Get tilt limit
		self.panLimit = float( getParameters(self.config, 'ChangeStateParameter', 'LimitPanAngleDegree'))

		self.confidenceThr = float( getParameters(self.config, 'ChangeStateParameter', 'BallConfidenceThreshold'))

		#	Get fx and fy from robot config
		self.fx = float( self.config[ "CameraParameters" ][ "fx" ] )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		
		#	re-initial numframe to detect ball
		self.numFrameNotDetectBall = 0
			
	def step( self ):
		
		#	Get vision msg
		visionMsg = self.rosInterface.visionManager

		localPosDict = self.rosInterface.local_map( reset = False ).postDict

		ballErrorY = 0

		#
		# This is logic for changing state.
		#
		if time.time() - self.timeStart > 2:
			if 'ball' in localPosDict.object_name:	

				idxBallLocalObj = localPosDict.object_name.index( 'ball' ) 
				localDistanceX = localPosDict.pos3D_cart[ idxBallLocalObj ].x
				localDistanceY = localPosDict.pos3D_cart[ idxBallLocalObj ].y
				thetaWrtRobotRad = localPosDict.pos2D_polar[ idxBallLocalObj ].y

				if localPosDict.object_confidence[ idxBallLocalObj ] < self.confidenceThr:
					#	it should switch to first state to find the ball
					self.SignalChangeSubBrain( self.lostBallState )

## NOTE : This case is not pratical since localMap not update object using visionMsg while robot is walking.			
				if math.fabs( thetaWrtRobotRad ) <= math.radians( self.smallTheta ):
					#	Back to previous state
					self.SignalChangeSubBrain( self.successState )

			else:
				self.SignalChangeSubBrain( self.lostBallState )

			#
			#	Rotate to ball
			#
			
			direction = 1 if thetaWrtRobotRad > 0 else -1
			if localDistanceX < 0.5 and math.fabs(thetaWrtRobotRad) < math.radians(60):

				self.rosInterface.LocoCommand(	velX = 0.0,
												velY = direction * self.omegaZ,
												omgZ = 0.0,
												command = 'OneStepWalk',
												commandType = 0,
												ignorable = False )

			else:
				self.rosInterface.LocoCommand(	velX = 0.0,
												velY = 0.0,
												omgZ = direction * self.omegaZ,
												command = 'OneStepWalk',
												commandType = 0,
												ignorable = False )
			self.time = time.time()

	def leaveStateCallBack( self ):

		self.stopRobotBehavior( )
		
	def stopRobotBehavior( self ):
		#	stop
		# self.rosInterface.LocoCommand( velX = 0.0,
		# 			       			   velY = 0.0,
		# 			       			   omgZ = 0.0,
		# 			       			   commandType = 0,
		# 			       			   ignorable = False )		
		self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   ignorable =  False )