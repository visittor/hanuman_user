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
		
	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		
		#	re-initial numframe to detect ball
		self.numFrameNotDetectBall = 0
			
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
			print thetaWrtRobotRad
			#	Check angle if not exceed 10 degrees
			if abs( thetaWrtRobotRad ) > math.radians( 10 ):
			
				self.rosInterface.LocoCommand(	velX = 0.0,
												velY = 0.0,
												omgZ = direction * 0.2,
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
		
			
		
