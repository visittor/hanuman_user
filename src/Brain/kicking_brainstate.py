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

class KickInMind( FSMBrainState ):

	def __init__( self, nextState = None ):

		#   set name
		super( KickInMind, self ).__init__( "KickInMind" )

		#   get next state
		self.nextState = nextState

	def firstStep( self ):
		
		rospy.loginfo( "Stop because robot cannot kick HAHAHAHAHAHA, fix model and how to detect ball first." )
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False ) 

	def step( self ):

		#	check ball
		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.nextState )

		#	get theta error from vision manger
		ballPolarArray = self.rosInterface.visionManager.ball_polar
		distanceBallToRobot = ballPolarArray[ 0 ]

		#   check distance 
		if distanceBallToRobot >= 0.35:

			self.SignalChangeSubBrain( self.nextState )
 
	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False ) 
		
 
