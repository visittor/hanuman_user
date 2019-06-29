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

DefaultLoopTimeToLookAtObject = 0.3
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

def getParameters( config, *arg ):

	try:
		for key in arg:

			config = config[ key ]

		return config
	except KeyError:

		config = _DEFAULT_CONFIG
		for key in arg:

			config = config[ key ]

		return config


########################################################
#
#	CLASS DEFINITIONS

class FindBall( FSMBrainState ):
	
	def __init__( self ):
		
		super( FindBall, self ).__init__( 'FindBall' )

		self.previousTime = time.time()

	def initialize( self ):

		#	Get time out from config
		self.scanBallPattern = self.config[ 'PanTiltPlanner' ][ 'ScanBallPattern' ]
		self.confidenceThr = float( getParameters(self.config, 'ChangeStateParameter', 'BallConfidenceThreshold'))

		self.lookBall = False

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )

		#	Call pattern
		self.rosInterface.Pantilt( command = 1, pattern = self.scanBallPattern )
		


	def step( self ):

		visionMsg = self.rosInterface.visionManager
		
		if 'ball' in visionMsg.object_name:

			self.previousTime = time.time()

			if not self.lookBall:
			
				self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
				self.lookBall = True			

		elif time.time() - self.previousTime > 3.0:
			
			if self.lookBall:
				self.rosInterface.Pantilt( command = 1, pattern = self.scanBallPattern )
				self.lookBall = False

main_brain = FindBall()