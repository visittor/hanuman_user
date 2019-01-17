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

	def __init__( self, nextState = None, previousState = None ):
		super( FollowBall, self ).__init__( 'FollowBall' )

		#	Set previous state
		self.previousState = previousState

		#	Set next state
		self.nextState = nextState

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

	def firstStep( self ):

		rospy.loginfo( "Enter follow ball state" )

		#	re-initialize time to look at the ball
		self.previousTime = 0.0

		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.nextState )

	def step( self ):

		#	initial current step time
		currentStepTime = time.time()

		if self.rosInterface.visionManager.ball_confidence == False:
			self.ChangeSubBrain( "StandStill" )
			self.SignalChangeSubBrain( self.previousState )
		
		#	look at the ball
		if currentStepTime - self.previousTime >= self.loopLookAtBall:

			#	command of pantilt to tracking the ball
			self.rosInterface.Pantilt( command = 2 )

			#	save time
			self.previousTime = currentStepTime
		
		print self.rosInterface.visionManager.ball_confidence
		#	get theta error from vision manger
		ballPolarArray = self.rosInterface.visionManager.ball_polar
		distanceBallToRobot = ballPolarArray[ 0 ]
		thetaBallRefToRobot = ballPolarArray[ 1 ]

		rospy.logdebug( "theta of the ball wrt. robot is {}".format( thetaBallRefToRobot ) )
		rospy.logdebug( "distance from robot to the ball is {}".format( distanceBallToRobot ) )

		#	check orientation of robot and the ball
		#	if angle of the ball wrt. robot more than 5, activate lococommand to turn it
		if abs( thetaBallRefToRobot ) >= np.deg2rad( 5 ):
			
			rospy.loginfo( "Robot is turn around" )

			#	get direction for rotate
			sign = thetaBallRefToRobot / abs( thetaBallRefToRobot )

			#	minus : turn left, positive : turn right
			if sign == 1:
				self.ChangeSubBrain( "TurnLeftToBall" )
			else:
				self.ChangeSubBrain( "TurnRightToBall" )
					
		else:
			
			rospy.loginfo( "Robot's position have aligned wrt the ball!!!!" )
			rospy.loginfo( "Robot is going straight to the ball!!!" )

			self.ChangeSubBrain( "ForwardToBall" )

		if distanceBallToRobot <= 0.2:

			rospy.loginfo( "Stop!! for keeping distance between robot and the ball" )

			#	call next state
			self.SignalChangeSubBrain( self.nextState )

