#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Kongkiat Rothomphiwat,
#                      Nasrun Hayeeyama
#

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os
import time
import rospy
import numpy as np

########################################################
#
#	LOCAL IMPORTS
#

from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

# from newbie_hanuman.msg import gameState, TeamInfo, RobotInfo

from gameControl_brainstate import InitialState, ReadyState, SetState, PlayState, FinishState, PernaltyState
from gameControl_brainstate import EnterField_dummy

from gameController.gamestate import GameState

from construct import Bytes

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

class AbnormalState( FSMBrainState ):
	def __init__( self ):
		super( AbnormalState, self ).__init__( "AbnormalState" )

	def firstStep( self ):
		rospy.logdebug( "Enter {} brainstate".format( self.name ) )

		self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   ignorable =  False )

	def step( self ):
		rospy.logdebug( "do Abnormal behaviour" )
		
		gameState = self.rosInterface.gameState
		secondState = gameState[ "secondary_state" ]
		secondStateInfo = gameState[ "secondary_state_info" ]

		secondStateInfoList = [ ord( i ) for i in secondStateInfo ]

		teamNumber = secondStateInfoList[ 0 ]

		behaviour = secondStateInfoList[ 1 ]

		if behaviour == 0:
			self.doStart()
		elif behaviour == 1:
			self.doEnd()
		elif behaviour == 2:
			self.doExecute()

	def doStart( self ):
		rospy.logdebug( "Start" )
	
	def doEnd( self ):
		rospy.logdebug( "End" )
	
	def doExecute( self ):
		rospy.logdebug( "Execute" )