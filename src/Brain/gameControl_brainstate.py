#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Kongkiat Rothomphiwat
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

from newbie_hanuman.msg import gameState, TeamInfo, RobotInfo

import numpy as np

import time
import rospy

from walk_around_fsmbrainstate import WalkAround

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
class InitialState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( InitialState, self ).__init__( "InitialState" )

	def firstStep( self ):

		print( "Entering initial state" )
		print( "Sit down" )
		
		#	terminate pantilt planner
		self.rosInterface.Pantilt( command=3 )
		
	def step( self ):
		# do command in this state
		print( "Waiting for ready signal" )
		
class ReadyState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( ReadyState, self ).__init__( "ReadyState" )

	def firstStep( self ):

		print( "Entering ready state" )
		print( "Stand up" )

	def step( self ):
		# do command in this state
		print( "Go to own half of the field" )


class SetState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( SetState, self ).__init__( "SetState" )

	def firstStep( self ):

		print( "Entering set state" )
		print( "Stop walking" )
		
		self.rosInterface.LocoCommand(	velX = 0.0,
 						velY = 0.0,
 						omgZ = 0.0,
 						commandType = 0,
 						ignorable = False )

	def step( self ):
		# do command in this state
		print( "Waiting for play signal" )

class PlayState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( PlayState, self ).__init__( "PlayState" )
		
		#	instan

	def firstStep( self ):

		print( "Entering play state" )
		
		

	def step( self ):
		# do command in this state
		print( "Play game!" )

class FinishState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( FinishState, self ).__init__( "FinishState" )

	def firstStep( self ):

		print( "Entering finish state" )
		print( "Stop walking" )
	def step( self ):
		# do command in this state
		print( "Finish game" )
