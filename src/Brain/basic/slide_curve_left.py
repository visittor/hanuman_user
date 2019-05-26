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

STATE_1 = 1
STATE_2 = 2

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

		self.state = None
	
	def initialize( self ):
		#	Get config

		self.velX = float( self.config[ 'VelocityParameter' ][ 'VelocityXSlideCurve' ] )
		self.velY = float( self.config[ 'VelocityParameter' ][ 'VelocityYSlideCurve' ] )
		self.omegaZ = float( self.config[ 'VelocityParameter' ][ 'OmegaZSlideCurve' ] )

		self.waitingTimeSlideCurve = float( self.config[ "VelocityParameter" ][ "WaitingTimeSlideCurve" ] )

	def firstStep( self ):

        #	command to slide curve
        #	TODO : Tune tomorrow
		# self.rosInterface.LocoCommand( velX = self.velX,
        #                                 velY = self.velY,
        #                                 omgZ = -1 * self.omegaZ,
        #                                 commandType = 0,
        #                                 ignorable = False )
 		
		# self.rosInterface.LocoCommand( velX = 0.5,
        #                                 velY = 0.0,
        #                                 omgZ = 0.0,
        #                                 commandType = 0,
        #                                 ignorable = False )
		#	get time step

		self.rosInterface.LocoCommand( velX = self.velX,
                                          velY = self.velY,
                                          omgZ = -1 * self.omegaZ,
                                          commandType = 0,
                                          ignorable = False )
		

		self.state = STATE_1

		self.previousTime = time.time()


	def step( self ):

		rospy.loginfo( "Vy : {}".format( self.velY ) )
		rospy.loginfo( "Omega : {}".format( self.omegaZ ) )
		
		#	get current time on these step
		currentTime = time.time()

		# if currentTime - self.previousTime >= 2 and self.state == STATE_1:

		# 	self.rosInterface.LocoCommand( command = "StandStill", commandType = 1 )

		# 	time.sleep( 1.0 )

		# 	self.rosInterface.LocoCommand( velX = self.velX,
        #                                   velY = self.velY,
        #                                   omgZ = -1 * self.omegaZ,
        #                                   commandType = 0,
        #                                   ignorable = False )

		# 	self.previousTime = time.time()
		# 	self.state = STATE_2
		
		# elif currentTime - self.previousTime >= 5 and self.state == STATE_2:

		# 	self.rosInterface.LocoCommand( command = "StandStill", commandType = 1 )

		# 	time.sleep( 1.0 )

		# 	# self.rosInterface.LocoCommand( velX = 0.5,
        #     #                             velY = 0.0,
        #     #                             omgZ = 0.0,
        #     #                             commandType = 0,
        #     #                             ignorable = False )

		# 	self.previousTime = time.time()
		# 	self.state = STATE_1


		
		#	TODO : Maybe change to radius
		if currentTime - self.previousTime >= self.waitingTimeSlideCurve:
			
			self.rosInterface.LocoCommand( velX = 0.0,
										   velY = 0.0,
										   omgZ = 0.0,
										   commandType = 0,
										   ignorable = False )
			
		# 	self.SignalChangeSubBrain( self.nextState )


main_brain = SlideCurve()
