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

from basicmove_brainstate import ForwardToBall, TurnLeftToBall, TurnRighToBall, StandStill, RightKick, LeftKick


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

class State:
	WALK = 1
	TURN_LEFT = 2
	TURN_RIGHT = 3
	STOP = 4
	KICK_RIGHT = 5
	KICK_LEFT = 6

class WalkAround( FSMBrainState ):

	def __init__( self, nextState = None ):

		super( WalkAround, self ).__init__( "Walk Around" )

		#
		#   initial instance basic move
		#
		goForward = ForwardToBall()
		turnLeft = TurnRighToBall()
		turnRight = TurnLeftToBall()
		standStill = StandStill()
		rightKick = RightKick()
		leftKick = LeftKick()

		#
		#   Add sub brain
		#
		self.addSubBrain( goForward )
		self.addSubBrain( turnLeft )
		self.addSubBrain( turnRight )
		self.addSubBrain( standStill )
		self.addSubBrain( rightKick )
		self.addSubBrain( leftKick )

		#   set first sub brain
		self.setFirstSubBrain( "StandStill" )

		#   define attribute init time
		self.initTime = None

		#   initial state attribute
		self.previousState = None
		self.state = None

		#   set time interval each state
		self.intervalForwardTime = 5.0 # second
		self.intervalTurnRightTime = 2.5 # second
		self.intervalTurnLeftTime = 2.5 # second
		self.intervalStopTime = 1.0 # second
		self.intervalKickingTime = 2.0

	def firstStep( self ):
		
		#   get time for first step
		self.initTime = time.time()

		#   log
		rospy.loginfo( "Start to turn around" )

		#   set first state to go forward
		self.ChangeSubBrain( "ForwardToBall" )
		self.state = State.WALK

	def step( self ):
		#   TODO : comment after coding finish


		#   Stamp time
		currentTime = time.time() - self.initTime

		if self.state == State.WALK and currentTime >= self.intervalForwardTime:
 			self.rosInterface.LocoCommand(	velX = 0.0,
 							velY = 0.0,
 							omgZ = 0.0,
 							commandType = 0,
 							ignorable = False )

			self.ChangeSubBrain( "TurnLeftToBall" )
			self.initTime = time.time()
			self.state = State.TURN_LEFT
		
		if self.state == State.TURN_LEFT and currentTime >= self.intervalTurnLeftTime:
			self.rosInterface.LocoCommand(	velX = 0.0,
 							velY = 0.0,
 							omgZ = 0.0,
 							commandType = 0,
 							ignorable = False )
			
			self.ChangeSubBrain( "RightKick" )
			self.initTime = time.time()
			self.state = State.KICK_RIGHT

		if self.state == State.KICK_RIGHT and currentTime >= self.intervalKickingTime:
			self.rosInterface.LocoCommand(	velX = 0.0,
 							velY = 0.0,
 							omgZ = 0.0,
 							commandType = 0,
 							ignorable = False )
			
			self.ChangeSubBrain( "LeftKick" )
			self.initTime = time.time()
			self.state = State.KICK_LEFT

		if self.state == State.KICK_LEFT and currentTime >= self.intervalForwardTime:
			self.rosInterface.LocoCommand(	velX = 0.0,
 							velY = 0.0,
 							omgZ = 0.0,
 							commandType = 0,
 							ignorable = False )
			
			self.ChangeSubBrain( "ForwardToBall" )
			self.initTime = time.time()
			self.state = State.WALK

			

			
		





