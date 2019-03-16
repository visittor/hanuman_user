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
import math

import time
import rospy

########################################################
#
#	GLOBALS
#

DefaultLoopTimeToLookAtObject = 1.5
DefaultObject = 'ball'
DefaultPantiltPattern = 'basic_pattern'

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

class TrackingBall( FSMBrainState ):


	def __init__( self, nextState = None ):

		super( TrackingBall, self ).__init__( 'TrackingBall' )
		self.nextState = nextState
		
		self.loopTimeLookAtBall = 1.5
		
		#	initial num frame for check it's the ball
		self.numFrame = 0
		
		#	initial object
		self.objectIndex = None
		
		#	time stamp
		self.stampTime = 0
		
	def firstStep( self ):
		'''
			first step before execute this brain
		'''

		rospy.loginfo( "Enter tracking ball state" )

		#	re-initial num frame for check it's the ball
		self.numFrame = 0
		
		while len( self.rosInterface.visionManager.object_name ) == 0: 
			pass
		
		#	get index object
		self.objectIndex = self.rosInterface.visionManager.object_name.index( DefaultObject )
		
		#	stamp time before enter step		
		self.stampTime = time.time()

	def step( self ):
		
		#	get current time
		currentStepTime = time.time()
		
		#	time remain 
		timeRemain = currentStepTime - self.stampTime
		
		if timeRemain >= self.loopTimeLookAtBall:
			
			if self.rosInterface.visionManager.object_confidence[ self.objectIndex ] >= 0.5:
				
				#	STARE!!!
				self.rosInterface.Pantilt( command = 2, pattern = DefaultObject )
				
				#	get theta ref to ball
				thetaWrtRobotDegree = self.rosInterface.visionManager.pos2D_polar[ self.objectIndex ].y
				
				#	get sign
				sign = abs( thetaWrtRobotDegree ) / thetaWrtRobotDegree
				
				self.rosInterface.LocoCommand(	velX = 0.0,
								velY = 0.0,
								omgZ = sign * 0.3,
								commandType = 0,
								ignorable = False )
				
				if abs( thetaWrtRobotDegree ) <= math.radians( 10 ):
					self.SignalChangeSubBrain( self.nextState )
				
			else:
				#	terminate pantilt
				self.rosInterface.Pantilt( command = 3 )
				self.rosInterface.Pantilt( command = 1, command = DefaultPantiltPattern )
				
				#	stop
				self.rosInterface.LocoCommand(	velX = 0.0,
								velY = 0.0,
								omgZ = 0.0,
								commandType = 0,
								ignorable = False )	

			self.stampTime = time.time()
