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

class TrackingBall( FSMBrainState ):
	
	#	state name
	FIND_BALL = 0
	LOOK_AT_BALL = 1

	def __init__( self, nextState = None ):

		super( TrackingBall, self ).__init__( 'TrackingBall' )
		self.nextState = nextState

		self.currentState = None

		self.loopTimeLookAtBall = 0.5

		#	initial num frame for check it's the ball
		self.numFrame = 0
		
	def firstStep( self ):
		'''
			first step before execute this brain
		'''

		rospy.loginfo( "Enter tracking ball state" )

		#	set first state : find ball
		self.currentState = self.FIND_BALL

		self.previousCommandTime = None
		self.rosInterface.Pantilt(	pattern="basic_pattern",
									command=1 )

	def step( self ):
		
		currentStepTime = time.time()

		#if timeStamp - self.initTime >= 3:
		if self.rosInterface.visionManager.ball_confidence == True and self.currentState == self.FIND_BALL:
			
			rospy.loginfo( "Detect the ball!!!!" )

			self.rosInterface.Pantilt( command = 3 )
			
			self.currentState = self.LOOK_AT_BALL

			self.previousCommandTime = currentStepTime

			self.SignalChangeSubBrain( self.nextState )

		if self.currentState == self.LOOK_AT_BALL and currentStepTime - self.previousCommandTime >= self.loopTimeLookAtBall:
            
            #   terminate current scan
			self.rosInterface.Pantilt( command = 2 )
			self.previousCommandTime = currentStepTime

            #   continue to find the ball
			if self.rosInterface.visionManager.ball_confidence == False:
				
				self.rosInterface.Pantilt(	pattern="basic_pattern",
											command=1 )
				
				self.currentState = self.FIND_BALL


