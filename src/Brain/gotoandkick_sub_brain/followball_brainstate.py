#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
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


class FollowBall( FSMBrainState ):

	def __init__( self, failState = "None", successState = "None", lostBallState = "None" ):
		
		super( FollowBall, self ).__init__( 'FollowBall' )
		
		self.failState = failState
		self.successState = successState
		
		self.lostBallState = lostBallState

		self.velX = None
		self.velY = None
		self.smallTheta = None
		self.distanceToKick = None

	def initialize( self ):
## NOTE : Visittor : Create default for these parameters --> Take it as an args of __init__.
		#	Get velocity of x and y
		self.velX_max = float( getParameters(self.config, 'VelocityParameter', 'VelocityXWhenFollowTheBall_max'))
		self.velX_min = float( getParameters(self.config, 'VelocityParameter', 'VelocityXWhenFollowTheBall_min'))
		self.velX_slope = float( getParameters(self.config, 'VelocityParameter', 'VelocityXWhenFollowTheBall_m'))
		self.velX_offset = float( getParameters(self.config, 'VelocityParameter', 'VelocityXWhenFollowTheBall_c'))

		self.omg_max = float( getParameters(self.config, 'VelocityParameter', 'OmegaZWhenRotateToBall_max'))
		self.omg_min = float( getParameters(self.config, 'VelocityParameter', 'OmegaZWhenRotateToBall_min'))
		self.omg_slope = float( getParameters(self.config, 'VelocityParameter', 'OmegaZWhenRotateToBall_m'))
		self.omg_offset = float( getParameters(self.config, 'VelocityParameter', 'OmegaZWhenRotateToBall_c'))

		self.velY = float( getParameters(self.config, 'VelocityParameter', 'VelocityYWhenFollowTheBall'))
		
		#	Get theta to change state
		self.smallTheta = float( getParameters(self.config, 'ChangeStateParameter', 'SmallDegreeToAlignTheBall'))

		#	Get neares distance before kick
		self.distanceToKick = float( getParameters(self.config, 'ChangeStateParameter', 'NearestDistanceFootballWrtRobot'))

		#	Get limit angle
		self.limitTiltAngle = float( getParameters(self.config, 'ChangeStateParameter', 'LimitTiltAngleDegree'))
		self.limitPanAngle = float( getParameters(self.config, 'ChangeStateParameter', 'LimitPanAngleDegree'))

		self.confidenceThr = float( getParameters(self.config, 'ChangeStateParameter', 'BallConfidenceThreshold'))

		#	Get fx and fy from robot config
		self.fy = float( self.config[ "CameraParameters" ][ "fy" ] )
		self.fx = float( self.config[ "CameraParameters" ][ "fx" ] )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )	
		
	def step( self ):
		
		#	Get current vision msg
		visionMsg = self.rosInterface.visionManager
		
		localPosDict = self.rosInterface.local_map( reset = False ).postDict

		pantiltJS = self.rosInterface.pantiltJS
		pantiltJS = { n:p  for n, p in zip( pantiltJS.name, pantiltJS.position ) }

		ballErrorY = 0

		#
		# This is logic for changing state.
		#

		if 'ball' in visionMsg.object_name:

			idxBallVisionObj = visionMsg.object_name.index( 'ball' )
			ballErrorX = visionMsg.object_error[ idxBallVisionObj ].x
			ballErrorY = visionMsg.object_error[ idxBallVisionObj ].y

			imgH = visionMsg.imgH
			fovHeight = 2 * np.arctan( 0.5 * imgH / self.fy )

			imgW = visionMsg.imgW
			fovWidth = 2 * np.arctan( 0.5 * imgW / self.fx )

			tiltAngle = ballErrorY * fovHeight / 2
			panAngle = ballErrorX * fovWidth / 2

			#	Get tilt angle from pan tilt motor
			currentTiltAngle = pantiltJS['tilt'] + tiltAngle
			currentPanAngle = pantiltJS['pan'] + panAngle

			if currentTiltAngle >= math.radians( self.limitTiltAngle ):
					rospy.loginfo( "Finish" )
					rospy.loginfo( "	Final Tilt angle : {}".format( math.degrees( currentTiltAngle ) ) )
					rospy.loginfo( "	Select side to kick : {}".format( self.getGlobalVariable( 'direction' ) ) )
					#	Change state to kick immedietly.
					self.SignalChangeSubBrain( self.successState )

			if math.fabs(currentPanAngle) >= math.radians( self.limitPanAngle ):
				self.SignalChangeSubBrain( self.lostBallState )

		#	Check confidence if model could detect ball
		if 'ball' in localPosDict.object_name:
			
			#	get distance and angle
			# distanceWrtBall = visionMsg.pos3D_cart[ idxBall ].x
			# sideDistanceWrtBall = visionMsg.pos3D_cart[ idxBall ].y

			idxBallLocalObj = localPosDict.object_name.index( 'ball' ) 
			localDistanceX = localPosDict.pos3D_cart[ idxBallLocalObj ].x
			localDistanceY = localPosDict.pos3D_cart[ idxBallLocalObj ].y
			thetaWrtRobotRad = localPosDict.pos2D_polar[ idxBallLocalObj ].y

			if localPosDict.object_confidence[ idxBallLocalObj ] < self.confidenceThr:
				#	it should switch to first state to find the ball
				self.SignalChangeSubBrain( self.lostBallState )
			
			if localDistanceX < self.distanceToKick:

				self.stopRobotBehavior()
				rospy.loginfo( "Finish" )
				rospy.loginfo( "	Distance after stop before kick : {} m".format( localDistanceX ) )
				rospy.loginfo( "	Select side to kick : {}".format( self.getGlobalVariable( 'direction' ) ) )

				self.SignalChangeSubBrain( self.successState )

## NOTE : This case is not pratical since localMap not update object using visionMsg while robot is walking.
			if abs( thetaWrtRobotRad ) > math.radians( self.smallTheta ) and localDistanceX > 0.5:
				
				#	Back to align the ball
				self.SignalChangeSubBrain( self.failState )

		else:		
			self.SignalChangeSubBrain( self.lostBallState )

		#
		#	Follow ball
		#

		idxBallLocalObj = localPosDict.object_name.index( 'ball' ) 
		localDistanceX = localPosDict.pos3D_cart[ idxBallLocalObj ].x
		localDistanceY = localPosDict.pos3D_cart[ idxBallLocalObj ].y

		direction = 1 if localDistanceY > 0 else -1
		
		self.setGlobalVariable( 'direction', direction )	
		
		velX = (localDistanceX * self.velX_slope) + self.velX_offset
		velX = max( self.velX_min, min( velX, self.velX_max ) )

		omgZ = (self.omg_slope*pantiltJS['pan']) + self.omg_offset
		omgZ = max( self.omg_min, min( omgZ, self.omg_max ) )

		self.rosInterface.LocoCommand( velX = velX,
					       			   velY = self.velY,
					       			   omgZ = omgZ,
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