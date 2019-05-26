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
	
	def initialize( self ):
		#	Get config

		self.velX = float( self.config[ 'VelocityParameter' ][ 'VelocityXSlideCurve' ] )
		self.velY = float( self.config[ 'VelocityParameter' ][ 'VelocityYSlideCurve' ] )
		self.omegaZ = float( self.config[ 'VelocityParameter' ][ 'OmegaZSlideCurve' ] )

		self.waitingTimeSlideCurve = float( self.config[ "VelocityParameter" ][ "WaitingTimeSlideCurve" ] )

	def firstStep( self ):

		#	Get direction from global variable
		direction = self.getGlobalVariable( 'curveSlideAngle' )
		
		if direction is not None:

			if abs(  math.degrees( direction ) ) < 30:
				self.SignalChangeSubBrain( self.nextState )

			direction = 1 if direction > 0 else -1

			#	command to slide curve
			#	TODO : Tune tomorrow
			self.rosInterface.LocoCommand( velX = self.velX,
										   velY = -1 * direction * self.velY,
										   omgZ = direction * self.omegaZ,
										   commandType = 0,
										   ignorable = False )

		else:

			dire = random.randint( 0, 1 )

			direction = -1 if dire == 0 else 1
			
			self.rosInterface.LocoCommand( velX = self.velX,
										   velY = -1*direction*self.velY,
										   omgZ = direction*self.omegaZ,
										   commandType = 0,
										   ignorable = False )

			
		
		#	get time step
		self.previousTime = time.time()
				
	def step( self ):
		
		#	get current time on these step
		currentTime = time.time()
		
		#	TODO : Maybe change to radius
		if currentTime - self.previousTime >= self.waitingTimeSlideCurve:
			
			self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   	   ignorable =  False )
			
			self.SignalChangeSubBrain( self.nextState )

# main_brain = SlideCurve()
