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
	
	def __init__( self, previousState = "None", nextState = "None" ):
		
		super( RotateToTheBall, self ).__init__( "RotateToTheBall" )
		
		self.nextState = nextState
		self.previousState = previousState

		self.numFrameNotDetectBall = None

		self.omegaZ = None
		self.smallTheta = None

		self.fy = None

	def initialize( self ):

		#	Get omega_z from config
		self.omegaZ = float( self.config[ 'VelocityParameter' ][ 'OmegaZWhenRotateToBall' ] )

		#	Get theta to change state
		self.smallTheta = float( self.config[ 'ChangeStateParameter' ][ 'SmallDegreeToAlignTheBall' ] )

		#	Get tilt limit
		self.tiltLimit = float( self.config[ 'PanTiltPlanner' ][ 'LimitTiltAngleDegree' ] )

		#	Get fx and fy from robot config
		self.fy = float( self.config[ "CameraParameters" ][ "fy" ] )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )

		self.rosInterface.Pantilt( command = 3 )	
		self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
		
		#	re-initial numframe to detect ball
		self.numFrameNotDetectBall = 0
			
	def step( self ):
		
		
		#	Get vision msg
		visionMsg = self.rosInterface.visionManager

		localPosDict = self.rosInterface.local_map( reset = False ).postDict

		ballErrorY = 0

		#	If detect
		if 'ball' in visionMsg.object_name:
		
			#	when vision manager found the ball
			self.numFrameNotDetectBall = 0

			idxBallVisionObj = visionMsg.object_name.index( 'ball' )
			
			#	Get polar coordinate
			thetaWrtRobotRad = visionMsg.pos2D_polar[ idxBallVisionObj ].y
			
			ballErrorY = visionMsg.object_error[ idxBallVisionObj ].y

			#	Get sign to rotate
			direction = 1 if thetaWrtRobotRad > 0 else -1
			
			#	Check angle if not exceed 10 degrees
			if abs( thetaWrtRobotRad ) > math.radians( self.smallTheta ):
			
				self.rosInterface.LocoCommand(	velX = 0.0,
												velY = 0.0,
												omgZ = direction * self.omegaZ,
												commandType = 0,
												ignorable = False )
			else:
				# self.rosInterface.LocoCommand(	velX = 0.0,
				# 								velY = 0.0,
				# 								omgZ = 0.0,
				# 								commandType = 0,
				# 								ignorable = False )
				
				self.SignalChangeSubBrain( self.nextState )
				
		else:	
			#	Increment when detect ball
			self.numFrameNotDetectBall += 1
			
			if self.numFrameNotDetectBall >= 5:
				#	terminate pantilt
				self.rosInterface.Pantilt( command = 3 )

				#	Stop locomotion
				self.rosInterface.LocoCommand( velX = 0.0,
							       velY = 0.0,
							       omgZ = 0.0,
							       commandType = 0,
							       ignorable = False )

				#	Back to previous state
				self.SignalChangeSubBrain( self.previousState )
		

		#	Get image width and height
		imgH = visionMsg.imgH
		fovHeight = 2 * np.arctan( 0.5 * imgH / self.fy )

		tiltAngle = ballErrorY * fovHeight / 2

		#	Get theta of tilt
		currentTiltAngle = self.rosInterface.pantiltJS.position[ 1 ] + tiltAngle
		if currentTiltAngle >= math.radians( self.tiltLimit ):

			self.SignalChangeSubBrain( self.nextState )		
		
