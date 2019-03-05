#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun (Nas) Hayeeyama
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

class RotateToTheBall( FSMBrainState ):
	
	def __init__( self, nextState = None, previousState = None ):
		
		#	set the name
		super( RotateToTheBall, self ).__init__( 'RotateToTheBall' )
		
		#	set next and previous state
		self.previousState = previousState
		self.nextState = nextState
		
		#	set time to look at the ball
		self.loopLookAtBall = 0.5
		
		#	initial theta attribure
		self.thetaBallRefToRobot = None
		
		#	initial previous time
		self.previousTime = None
		
	def firstStep( self ):
		
		rospy.logdebug( "Enter to rotate the ball" )
		
		#	back to findball when lose the ball
		if self.rosInterface.visionManager.ball_confidence == False:
			self.rosInterface.LocoCommand(	velX = 0.0,
							velY = 0.0,
							omgZ = 0.0,
							commandType = 0,
							ignorable = False )
			
			self.SignalChangeSubBrain( self.previousState )
		
		#	get theta of the ball respect to robot
		self.thetaBallRefToRobot = self.rosInterface.visionManager.ball_polar[ 1 ]
		
		#	re-initialize previous time on first step
		self.previousTime = 0.0	
		
		#	command to rotate
		sign = self.thetaBallRefToRobot / abs( self.thetaBallRefToRobot )
		self.rosInterface.LocoCommand(	velX = 0.0,
						velY = 0.0,
						omgZ = sign * 0.3,
						commandType = 0,
						ignorable = False )
						
	def step( self ):
		
		#	get current step time
		currentStepTime = time.time()
		
		#	back to findball when lose the ball
		if self.rosInterface.visionManager.ball_confidence == False:
			
			self.rosInterface.LocoCommand(	velX = 0.0,
							velY = 0.0,
							omgZ = 0.0,
							commandType = 0,
							ignorable = False )
			
			self.SignalChangeSubBrain( self.previousState )
		
		#	check angle 
		self.thetaBallRefToRobot = self.rosInterface.visionManager.ball_polar[ 1 ]
		
		rospy.logdebug( "theta ref to the robot : {} degree".format( np.rad2deg( self.thetaBallRefToRobot ) ) )
		
		#	look at the ball
		if currentStepTime - self.previousTime >= self.loopLookAtBall:

			#	command of pantilt to tracking the ball
			self.rosInterface.Pantilt( command = 2 )

			#	save time
			self.previousTime = currentStepTime
		
		#	when get to desire degree
		if abs( self.thetaBallRefToRobot ) <= np.deg2rad( 10 ):
			
			self.rosInterface.LocoCommand(	velX = 0.0,
							velY = 0.0,
							omgZ = 0.0,
							commandType = 0,
							ignorable = False )
			
			if self.nextState is not None:
				#	go to next state
				self.SignalChangeSubBrain( self.nextState )
			else:
				rospy.logdebug( "Finish to align" )
