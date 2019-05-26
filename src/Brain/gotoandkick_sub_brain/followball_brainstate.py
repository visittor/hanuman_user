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

	def __init__( self, previousState = "None", nextState = "None", findBallState = "None" ):
		
		super( FollowBall, self ).__init__( 'FollowBall' )
		
		self.previousState = previousState
		self.nextState = nextState
		
		self.findBallState = findBallState
		
		self.numFrameNotDetectBall = None

		self.velX = None
		self.velY = None
		self.smallTheta = None
		self.distanceToKick = None

	def initialize( self ):
		
		#	Get velocity of x and y
		self.velX = float( self.config[ 'VelocityParameter' ][ 'VelocityXWhenFollowTheBall' ] )
		self.velY = float( self.config[ 'VelocityParameter' ][ 'VelocityYWhenFollowTheBall' ] )
		
		#	Get theta to change state
		self.smallTheta = float( self.config[ 'ChangeStateParameter' ][ 'SmallDegreeToAlignTheBall' ] )

		#	Get neares distance before kick
		self.distanceToKick = float( self.config[ 'ChangeStateParameter' ][ 'NearestDistanceFootballWrtRobot' ] )

		#	Get limit angle
		self.limitTiltAngle = float( self.config[ 'PanTiltPlanner' ][ 'LimitTiltAngleDegree' ] )

		#	Get fx and fy from robot config
		self.fy = float( self.config[ "CameraParameters" ][ "fy" ] )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )	
		
		#	re-initial num frame that not detect ball
		self.numFrameNotDetectBall = 0

		# self.rosInterface.Pantilt( command = 3 )
			
		# self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
		
	def step( self ):
		
		#	Get current vision msg
		visionMsg = self.rosInterface.visionManager
		
		localPosDict = self.rosInterface.local_map( reset = False ).postDict
		
		ballErrorY = 0

		if 'ball' in visionMsg.object_name:

			idxBallVisionObj = visionMsg.object_name.index( 'ball' )
			ballErrorY = visionMsg.object_error[ idxBallVisionObj ].y

		imgH = visionMsg.imgH
		fovHeight = 2 * np.arctan( 0.5 * imgH / self.fy )

		tiltAngle = ballErrorY * fovHeight / 2

		#	Get tilt angle from pan tilt motor
		currentTiltAngle = self.rosInterface.pantiltJS.position[ 1 ] + tiltAngle
		if currentTiltAngle >= math.radians( self.limitTiltAngle ):

				rospy.loginfo( "	Final Tilt angle : {}".format( math.degrees( currentTiltAngle ) ) )
				rospy.loginfo( "	Select side to kick : {}".format( self.getGlobalVariable( 'direction' ) ) )
				#	Change state to kick immedietly.
				self.stopRobotBehavior()

				self.SignalChangeSubBrain( self.nextState )

		if 'ball' in visionMsg.object_name:
			idxBallVisionObj = visionMsg.object_name.index( 'ball' )
			thetaWrtBall = visionMsg.pos2D_polar[ idxBallVisionObj ].y
			if abs( thetaWrtBall ) > math.radians( self.smallTheta ):
				
				#	Back to align the ball
				self.SignalChangeSubBrain( self.previousState )

		#
		#	Follow ball
		#

		#	Check confidence if model could detect ball
		if 'ball' in localPosDict.object_name:
			
			#	get distance and angle
			# distanceWrtBall = visionMsg.pos3D_cart[ idxBall ].x
			# sideDistanceWrtBall = visionMsg.pos3D_cart[ idxBall ].y

			idxBallLocalObj = localPosDict.object_name.index( 'ball' ) 
			localDistanceX = localPosDict.pos3D_cart[ idxBallLocalObj ].x
			localDistanceY = localPosDict.pos3D_cart[ idxBallLocalObj ].y

			if localDistanceX >= self.distanceToKick:
				
				#	Get side to kick in kicking brain state
				direction = 1 if localDistanceY > 0 else -1
				
				self.setGlobalVariable( 'direction', direction )	
			
				self.rosInterface.LocoCommand( velX = self.velX,
							       			   velY = self.velY,
							       			   omgZ = 0.0,
							       			   commandType = 0,
							       			   ignorable = False )
			else:
	
				self.stopRobotBehavior()

				rospy.loginfo( "Finish" )
				rospy.loginfo( "	Distance after stop before kick : {} m".format( localDistanceX ) )
				rospy.loginfo( "	Select side to kick : {}".format( self.getGlobalVariable( 'direction' ) ) )

				self.SignalChangeSubBrain( self.nextState )
				
			
		else:
			#	it should switch to first state to find the ball
			self.stopRobotBehavior()
			
			self.SignalChangeSubBrain( self.findBallState )
			
			
	def stopRobotBehavior( self ):
		
		#	terminate pantilt
		self.rosInterface.Pantilt( command = 3 )
			
		#	stop
		# self.rosInterface.LocoCommand( velX = 0.0,
		# 			       			   velY = 0.0,
		# 			       			   omgZ = 0.0,
		# 			       			   commandType = 0,
		# 			       			   ignorable = False )		
		self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   ignorable =  False )