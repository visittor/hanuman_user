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

		rospy.logdebug( "Entering initial state" )
		rospy.logdebug( "Sit down" )
		
	def step( self ):
		# do command in this state
		rospy.logdebug( "Waiting for ready signal" )
		
class ReadyState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( ReadyState, self ).__init__( "ReadyState" )

	def firstStep( self ):

		rospy.logdebug( "Entering ready state" )
		rospy.logdebug( "Stand up" )

	def step( self ):
		# do command in this state
		rospy.logdebug( "Go to own half of the field" )


class SetState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( SetState, self ).__init__( "SetState" )

	def firstStep( self ):

		rospy.logdebug( "Entering set state" )
		rospy.logdebug( "Stop walking" )
		

	def step( self ):
		# do command in this state
		rospy.logdebug( "Waiting for play signal" )

class PlayState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( PlayState, self ).__init__( "PlayState" )


	def firstStep( self ):

		rospy.logdebug( "Entering play state" )
		
	def step( self ):
		# do command in this state
		rospy.logdebug( "Play game!" )

class FinishState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( FinishState, self ).__init__( "FinishState" )

	def firstStep( self ):

		rospy.logdebug( "Entering finish state" )
		
	def step( self ):
		# do command in this state
		rospy.logdebug( "Finish game" )

class PernaltyState( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( PernaltyState, self ).__init__( "PernaltyState" )

	def firstStep( self ):

		rospy.logdebug( "Entering pernalty state TT" )
	
	def step( self ):
		rospy.logdebug( "TT" )