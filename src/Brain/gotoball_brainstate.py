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

from basicmove_brainstate import ForwardToBall, TurnLeftToBall, TurnRighToBall
from basicmove_brainstate import StandStill

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

class FollowBall( FSMBrainState ):

	def __init__( self, kickingState = None, trackingState = None, alignState = None  ):
		super( FollowBall, self ).__init__( 'FollowBall' )

		#	Set previous state
		self.trackingState = trackingState

		#	Set next state
		self.kickingState = kickingState
		
		#	Set align state
		self.alignState = alignState

		#   define instance of basic move brain state
		moveForward = ForwardToBall()
		turnRight = TurnRighToBall()
		turnLeft = TurnLeftToBall()
		standStill = StandStill()

		#   add substate from basic move brain state
		self.addSubBrain( standStill )
		self.addSubBrain( moveForward )
		self.addSubBrain( turnRight )
		self.addSubBrain( turnLeft )

		#	set first sub brain to initial state
		self.setFirstSubBrain( "StandStill" )

		#	set time to look at the ball
		self.loopLookAtBall = 0.5
		self.previousTime = 0.0
		
		#	initial flag to set side to align
		#	Right side is 1
		#	LeftSide is -1
		self.ballSide = None
		self.yMagnitude = None
		
		#	get distant of the ball
		self.xMagnitude = None
		
		#	theta of the ball w.r.t robot
		self.thetaBallWrtRobot = None

	def firstStep( self ):

		rospy.loginfo( "Enter follow ball state" )

		#	re-initialize time to look at the ball
		self.previousTime = 0.0

		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.trackingState )
		
		#	get position from y-axis
		self.yMagnitude = self.rosInterface.visionManager.ball_cart[ 1 ]
		
		#	set side to align
		self.ballSide = 1 if self.yMagnitude > 0 else -1
		
		
	def step( self ):

		#	initial current step time
		currentStepTime = time.time()

		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.trackingState )
		
		#	get position from y-axis
		self.yMagnitude = self.rosInterface.visionManager.ball_cart[ 1 ]
		self.xMagnitude = self.rosInterface.visionManager.ball_cart[ 0 ]
		
		#	get theta of ball wrt robot
		self.thetaBallWrtRobot = self.rosInterface.visionManager.ball_polar[ 1 ]
		
		#	look at the ball
		if currentStepTime - self.previousTime >= self.loopLookAtBall:

			#	command of pantilt to tracking the ball
			self.rosInterface.Pantilt( command = 2 )

			#	save time
			self.previousTime = currentStepTime
		
		rospy.logdebug( "X distance : {}".format( self.xMagnitude ) )
		rospy.logdebug( "Y distance : {}".format( self.yMagnitude ) )
		rospy.logdebug( "theta of robot wrt : {}".format( self.thetaBallWrtRobot ) )
		
		
		#	change state when robot is outside theta threshold
		if abs( self.thetaBallWrtRobot ) >= np.deg2rad( 15 ):
			self.SignalChangeSubBrain( self.alignState )
		
		#	Probably zig-zag ?
		
		#	check distant on y-axis
#		if abs( self.yMagnitude ) > 0.5:
#			
#			self.rosInterface.LocoCommand(	velX = 0.2,
#							velY = self.ballSide * 0.2,
#							omgZ = 0,
#							commandType = 0,
#							ignorable = False )
#		
#		elif abs( self.yMagnitude ) < 0.2:
#			
#			self.rosInterface.LocoCommand(	velX = 0.2,
#							velY = -1 * self.ballSide * 0.2,
#							omgZ = 0,
#							commandType = 0,
#							ignorable = False )
		
		self.rosInterface.LocoCommand(	velX = 0.2,
						velY = 0,
						omgZ = 0,
						commandType = 0,
						ignorable = False )
		
		#	arrival at desire position			
		if self.xMagnitude <= 0.25:
		
			#	STOPPP
			self.rosInterface.LocoCommand(	velX = 0,
							velY = 0,
							omgZ = 0,
							commandType = 0,
							ignorable = False )
			
			self.SignalChangeSubBrain( self.kickingState )				
		
							
							
		

