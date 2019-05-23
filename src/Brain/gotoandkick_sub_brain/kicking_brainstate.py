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
		
		self.previousTime = previousState

		self.velX = None
		self.waitTime = None

	def initialize( self ):

		#	Get time and velocity x
		self.velX = float( self.config[ 'VelocityParameter' ][ 'VelocityXWhenStepToKick' ] )
		self.waitTime = float( self.config[ 'VelocityParameter' ][ 'WaitingTimeAfterStep' ] )


	def firstStep( self ):

		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		
		self.rosInterface.LocoCommand( velX = self.velX ,
									   velY = 0.0,
									   omgZ = 0.0,
									   command = 'OneStepWalk',
									   commandType = 0 )
		
		time.sleep( 1.0 )

		self.previousTime  = time.time()
	
	
	def step( self ):
		
		currentTime = time.time()

		rospy.loginfo( "Time to kick : {}".format( currentTime - self.previousTime ) )
		
		if currentTime - self.previousTime > self.waitTime:
			
			#	Get side to kick
			direction = self.getGlobalVariable( 'direction' )
			#direction = 1
			
			if direction > 0:
				self.rosInterface.LocoCommand( command = "LeftKick", commandType = 1 )
			else:
				self.rosInterface.LocoCommand( command = "RightKick", commandType = 1 )

			time.sleep( 3 )
			self.rosInterface.LocoCommand( velX = 0.0,
									   	   velY = 0.0,
									   	   omgZ = 0.0,
									   	   commandType = 0 )

			self.SignalChangeSubBrain( self.nextState ) 
		
# main_brain = KickTheBall()