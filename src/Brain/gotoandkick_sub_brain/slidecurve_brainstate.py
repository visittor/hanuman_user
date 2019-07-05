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

import random

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


class SlideCurve( FSMBrainState ):

	def __init__( self, nextState = "None" ):
		
		super( SlideCurve, self ).__init__( 'SlideCurve' )
		
		self.nextState = nextState
		self.previousTime = None

		#	Initial velocity parameter
		self.velX = None
		self.velY = None
		self.omegaZ = None
	
		self.actualWaitingTime = 0

	def initialize( self ):
		#	Get config

		self.velX = float( getParameters(self.config, 'VelocityParameter', 'VelocityXSlideCurve'))
		self.velY = float( getParameters(self.config, 'VelocityParameter', 'VelocityYSlideCurve'))
		self.omegaZ = float( getParameters(self.config, 'VelocityParameter', 'OmegaZSlideCurve' ))

		self.waitingTime = float( getParameters(self.config, "VelocityParameter", "WaitingTimeSlideCurve"))

	def firstStep( self ):

		#	Get direction from global variable
		direction = self.getGlobalVariable( 'curveSlideAngle' )

		direction = direction % (2*np.pi)
		direction = np.pi - direction

		directionDegrees = int(math.degrees( math.fabs(direction) ))

		self.actualWaitingTime = self.waitingTime * (directionDegrees / 20)
		
		if direction is not None:

			# if abs(  math.degrees( direction ) ) < 30:
			# 	self.SignalChangeSubBrain( self.nextState )

			direction = 1 if direction > 0 else -1

			#	command to slide curve
			#	TODO : Tune tomorrow
			self.rosInterface.LocoCommand( velX = 0.0,
										   velY = -1.0 * direction,
										   omgZ = 0.0,
										   commandType = 0,
										   command = 'SlideCurve',
										   ignorable = False )

		else:
			
			self.rosInterface.LocoCommand( velX = 0.0,
										   velY = 0.0,
										   omgZ = 0.0,
										   commandType = 0,
										   ignorable = True )
		
		#	get time step
		self.previousTime = time.time()
				
	def step( self ):
		
		#	get current time on these step
		currentTime = time.time()
		
		#	TODO : Maybe change to radius
		if currentTime - self.previousTime >= self.actualWaitingTime:
			
			self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   	   ignorable =  False )
			
			self.SignalChangeSubBrain( self.nextState )

# main_brain = SlideCurve()
