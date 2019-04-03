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


class SlideCurve( FSMBrainState ):

	def __init__( self, nextState = "None" ):
		
		super( SlideCurve, self ).__init__( 'SlideCurve' )
		
		self.nextState = nextState
		self.previousTime = None
		
		
	def firstStep( self ):
		
		#	command to slide curve
		#	TODO : Tune tomorrow
		self.rosInterface.LocoCommand( velX = 0.0,
						velY = 0.0,
						omgZ = 0.0,
						commandType = 0,
						ignorable = False )
		
		
		#	get time step
		self.previousTime = time.time()
				
	def step( self ):
		
		#	get current time on these step
		currentTime = time.time()
		
		#	TODO : Maybe change to radius
		if currentTime - self.previousTime >= 5.0:
			
			self.rosInterface.LocoCommand( velX = 0.0,
							velY = 0.0,
							omgZ = 0.0,
							commandType = 0,
							ignorable = False )
			
			self.SignalChangeSubBrain( self.nextState )

main_brain = SlideCurve()
