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
	TURN = 7

class WalkAround( FSMBrainState ):

	def __init__( self, nextState = None ):

		super( WalkAround, self ).__init__( "WalkAround" )

		#
		#   initial instance basic move
		#
		goForward = ForwardToBall()
		turnLeft = TurnLeftToBall()
		turnRight = TurnRighToBall()
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
		self.stateTurn = None

		#   set time interval each state
		self.intervalForwardTime = 10.0 # second
		self.intervalTurnRightTime = 5.0# second
		self.intervalTurnLeftTime = 5.0 # second
		self.intervalStopTime = 1.0 # second
		self.intervalKickingTime = 3.0

	def firstStep( self ):
		
		#   get time for first step
		self.initTime = time.time()

		#   log
		rospy.loginfo( "Start to turn around" )

		#   set first state to go forward
		self.ChangeSubBrain( "ForwardToBall" )
		self.state = State.WALK
		self.stateTurn = State.TURN_LEFT
		
		#	command to pan and tilt
		self.rosInterface.Pantilt(	pattern="basic_pattern",
						command=1 )

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

			#	Change state
			#	toggle state between turn left and turn right
			if self.stateTurn == State.TURN_LEFT:
				
				#	change sub brain
				self.ChangeSubBrain( "TurnLeftToBall" )
				
				#	set next turn to oppostite direction
				self.stateTurn = State.TURN_RIGHT
			
			elif self.stateTurn == State.TURN_RIGHT:
				
				#	change sub brain
				self.ChangeSubBrain( "TurnRightToBall" )
				
				#	set next turn to oppostite direction
				self.stateTurn = State.TURN_LEFT
			
			
			#	re-initialize time
			self.initTime = time.time()
			
			#	set state
			self.state = State.TURN
		
		elif self.state == State.TURN and currentTime >= self.intervalTurnLeftTime:
			self.rosInterface.LocoCommand(	velX = 0.0,
 							velY = 0.0,
 							omgZ = 0.0,
 							commandType = 0,
 							ignorable = False )
			
			self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
							position=[ 0, 0 ],
							command=0 )
			#	Change state			
			self.ChangeSubBrain( "RightKick" )
			
			#	re-initialize time
			self.initTime = time.time()
			
			#	set state
			self.state = State.KICK_RIGHT
			
		elif self.state == State.KICK_RIGHT and currentTime >= self.intervalKickingTime:
			self.rosInterface.LocoCommand(	velX = 0.0,
 							velY = 0.0,
 							omgZ = 0.0,
 							commandType = 0,
 							ignorable = False )
							
			self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
							position=[ 0, 0 ],
							command=0 )
			
			self.ChangeSubBrain( "LeftKick" )
			self.initTime = time.time()
			self.state = State.KICK_LEFT

		elif self.state == State.KICK_LEFT and currentTime >= self.intervalKickingTime:
			self.rosInterface.LocoCommand(	velX = 0.0,
 							velY = 0.0,
 							omgZ = 0.0,
 							commandType = 0,
 							ignorable = False )
			
			self.rosInterface.Pantilt(	pattern="basic_pattern",
							command=1 )
			
			self.ChangeSubBrain( "ForwardToBall" )
			self.initTime = time.time()
			self.state = State.WALK

			

class MainBrain( FSMBrainState ):
	# def firstStep(self):
	# 	self.rosInterface.Pantilt(	pattern="basic_pattern",
	# 								command=1)

	def end(self):

		self.rosInterface.Pantilt( command = 3 )
		self.rosInterface.LocoCommand(	velX = 0.0,
						velY = 0.0,
						omgZ = 0.0,
						commandType = 0,
						ignorable = False )
		time.sleep(1)			
		

# walkAroundState = WalkAround()

# main_brain = MainBrain( "main_brain" )
# main_brain.addSubBrain( walkAroundState )
# main_brain.setFirstSubBrain( "WalkAround" ) 



