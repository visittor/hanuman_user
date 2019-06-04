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
class KickTheBall( FSMBrainState ):
	
	
	def __init__( self, nextState = "None", previousState = "None" ):
	
		#	set name
		super( KickTheBall, self ).__init__( "KickTheBall" )
 		
		#	get next state
		self.nextState = nextState
		
		self.previousTime = previousState

		self.velX = None
		self.waitTime = None

		self.direction = 1

	def initialize( self ):

		#	Get time and velocity x
		self.velX = float( getParameters(self.config, 'VelocityParameter', 'VelocityXWhenStepToKick'))
		self.waitTime = float( getParameters(self.config, 'VelocityParameter', 'WaitingTimeAfterStep'))

		self.tiltAngle = float( getParameters(self.config, 'PanTiltPlanner', 'TiltAngleForLookAtFoot'))

	def firstStep( self ):

		rospy.loginfo( "Enter {} brainstate".format( self.name ) )

		self.direction = 1
		
		self.rosInterface.LocoCommand( velX = self.velX ,
									   velY = 0.0,
									   omgZ = 0.0,
									   command = 'OneStepWalk',
									   commandType = 0,
									   ignorable = False )
		
		time.sleep( 2.0 )

## NOTE : What is this for
		self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
									position=[ 0.0, math.radians( self.tiltAngle ) ],
									command=0,
									velocity=[100, 100] )

		# if direction > 0:
		# 	self.rosInterface.LocoCommand( command = "LeftKick", commandType = 1 )
		# else:
		# 	self.rosInterface.LocoCommand( command = "RightKick", commandType = 1 )

		self.previousTime  = time.time()
	
	def step( self ):
		
		currentTime = time.time()

		# rospy.loginfo( "Time to kick : {}".format( currentTime - self.previousTime ) )
		visionMsg = self.rosInterface.visionManager

		if 'ball' in visionMsg.object_name:
			idxBallVisionObj = visionMsg.object_name.index( 'ball' )
			errorX = visionMsg.object_error[ idxBallVisionObj ].x

			self.direction = 1 if errorX > 0.0 else -1
			rospy.loginfo( "I SHALL KICK {}.".format( 'LEFT' if self.direction > 1 else "RIGHT" ))
		
		elif 'kicking_side' in visionMsg.object_name:
			idxBallVisionObj = visionMsg.object_name.index( 'kicking_side' )
			errorX = visionMsg.object_error[ idxBallVisionObj ].x
			self.direction = 1 if errorX > 0.0 else -1
			rospy.loginfo( "I SHALL KICK {}.".format( 'LEFT' if self.direction > 1 else "RIGHT" ))
		
		if currentTime - self.previousTime > self.waitTime:

			self.rosInterface.local_map( reset = True ).postDict
			
			#	Get side to kick
			direction = self.getGlobalVariable( 'direction' )
			#direction = 1
			
			if self.direction > 0:
				self.rosInterface.LocoCommand( command = "LeftKick", commandType = 1 )
			else:
				self.rosInterface.LocoCommand( command = "RightKick", commandType = 1 )

			time.sleep( 3 )
			self.rosInterface.LocoCommand( velX = 0.0,
									   	   velY = 0.0,
									   	   omgZ = 0.0,
									   	   commandType = 0 )

			self.rosInterface.local_map( reset = True )

			self.SignalChangeSubBrain( self.nextState ) 
		
# main_brain = KickTheBall()