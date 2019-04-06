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


class KickTheBall( FSMBrainState ):
	
	
	def __init__( self, nextState = "None", previousState = "None" ):
	
		#	set name
		super( KickTheBall, self ).__init__( "KickTheBall" )
 		
		#	get next state
		self.nextState = nextState
		
		self.previousTime = None
			
	def firstStep( self ):

		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		
		self.rosInterface.LocoCommand( velX = 0.4,
						velY = 0.0,
						omgZ = 0.0,
						command = 'oneStepWalk' )
		
		self.previousTime  = time.time()
	
	
	def step( self ):
		
		currentTime = time.time()
		
		if currentTime - self.previousTime > 3:
			
			#	Get side to kick
			direction = self.getGlobalVariable( 'direction' )
			
			if direction > 0:
				self.rosInterface.LocoCommand( command = "LeftKick", commandType = 1 )
			else:
				self.rosInterface.LocoCommand( command = "RightKick", commandType = 1 )
		
