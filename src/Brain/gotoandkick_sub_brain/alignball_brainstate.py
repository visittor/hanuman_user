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
		
		self.numFrameDetectBall = None
		self.numFrameNotDetectBall = None
		
		self.sumAngleWrtRobot = None
		self.meanAngleWrtRobot = None
		
		self.direction = None
		
	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		
		visionMsg = self.rosInterface.visionManager
		
		idxBallObj = visionMsg.object_name.index( 'ball' )
		
		#	terminate when robot cannot detect ball
		if visionMsg.object_confidence[ idxBallObj ] < 0.5:
			
			self.rosInterface.Pantilt( command = 3 )
			
		#	re-initial numframe to detect ball
		self.numFrameDetectBall = 0
		self.numFrameNotDetectBall = 0

		#	re-initial sum and mean before step
		self.sumAngleWrtRobot = 0.0
		self.meanAngleWrtRobot = 0.0
			
	def step( self ):
		
		
		#	Get vision msg
		visionMsg = self.rosInterface.visionManager
		
		idxBallObj = visionMsg.object_name.index( 'ball' )
		
		#	If detect
		if visionMsg.object_confidence[ idxBallObj ] > 0.5:
		
			#	when vision manager found the ball
			self.numFrameNotDetectBall = 0
			
			#	Get polar coordinate
			thetaWrtRobotRad = visionMsg.pos2D_polar[ idxBallObj ].y
			
			#	Get sign to rotate
			direction = 1 if thetaWrtRobotRad > 0 else -1
			
			if self.numFrameDetectBall < 20:
				self.numFrameDetectBall += 1
		
						
			#	Get polar coordinate
			thetaWrtRobotRad = visionMsg.pos2D_polar[ idxBallObj ].y
			
			#	Get distance ball wrt robot
			distanceWrtRobot = visionMsg.pos3D_cart[ idxBallObj ].x

			#	Approximate angle of ball wrt robot
			self.sumAngleWrtRobot += thetaWrtRobotRad
			
			#	Find mean
			self.meanAngleWrtRobot = float( self.sumAngleWrtRobot ) / self.numFrameDetectBall
			
			rospy.logdebug( "Detect angle : {}".format( thetaWrtRobotRad ) )
			rospy.logdebug( "Approximate angle : {}".format( self.meanAngleWrtRobot ) )
			
			#	Check angle if not exceed 10 degrees
			if self.meanAngleWrtRobot > abs( math.radians( 10 ) ):
			
			
				self.rosInterface.LocoCommand(	velX = 0.0,
								velY = 0.0,
								omgZ = direction * 0.2,
								commandType = 0,
								ignorable = False )
			else:
				self.rosInterface.LocoCommand(	velX = 0.0,
								velY = 0.0,
								omgZ = 0.0,
								commandType = 0,
								ignorable = False )
				
				self.SignalChangeSubBrain( self.nextState )
				
		else:	
			#	Increment when detect ball
			self.numFrameNotDetectBall += 1
			
			if self.numFrameNotDetectBall >= 3:
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
		
			
		
