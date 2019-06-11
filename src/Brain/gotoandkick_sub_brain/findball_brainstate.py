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

DistanceThreshold = 0.30

########################################################
#
#	EXCEPTION DEFINITIONS
#

########################################################
#
#	HELPER FUNCTIONS
#

########################################

#
#	CLASS DEFINITIONS
#

class FindBall( FSMBrainState ):
	
	def __init__( self, nextState = "None" ):

		
		super( FindBall, self ).__init__( "FindBall" )

		#	Intial attribute for time ou

		self.findBallStateTimeOut = None

		self.stepDirection = [ 1, -1, -1, 1 ]
		self.stepIndex = 0

	def initialize( self ):

		#	Get time out from config
		self.findBallStateTimeOut = float( self.config[ 'ChangeStateParameter' ][ 'FindballStateTimeOut' ] )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		self.stepIndex = 0
		self.timeStart = time.time( )

	def step( self ):

		if time.time( ) - self.timeStart > self.findBallStateTimeOut:

			direction = self.stepDirection[ self.stepIndex ]

			self.rosInterface.LocoCommand(	velX = 0.0,
											velY = 0.0,
											omgZ = direction * 0.8,
											command = 'OneStepWalk',
											commandType = 0,
											ignorable = False )

			self.stepIndex = ( self.stepIndex + 1 ) % len( self.stepDirection )
			self.timeStart = time.time( )

	def leaveStateCallBack( self ):

		self.stopRobotBehavior( )
		
	def stopRobotBehavior( self ):
		#	stop
		self.rosInterface.LocoCommand( velX = 0.0,
					       			   velY = 0.0,
					       			   omgZ = 0.0,
					       			   commandType = 0,
					       			   ignorable = False )		
		# self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
		# 							   ignorable =  False )