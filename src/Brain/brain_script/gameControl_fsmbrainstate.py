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

from gameControl_brainstate import InitialState, ReadyState, SetState, PlayState, FinishState, PenaltyState

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

class Controller( FSMBrainState ):

	def __init__( self, initialState, readyState, setState, playState, finishState, penaltyState ):
		
		super( Controller, self ).__init__( "Controller" )

		self.playerNumber = None
		self.teamNumber = None

		self.addSubBrain( initialState, "InitialState" )
		self.addSubBrain( readyState, "ReadyState" )
		self.addSubBrain( setState, "SetState" )
		self.addSubBrain( playState, "PlayState" )
		self.addSubBrain( finishState, "FinishState" )

		self.addSubBrain( penaltyState, "PenaltyState" )

	def firstStep( self ):
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )

	def initialize( self ):

		self.teamNumber = int( self.config[ "GameControllerParameter" ][ "Team" ] )
		self.playerNumber = int( self.config[ "GameControllerParameter" ][ "Player" ] )

	def step( self ):
		
		gameState = self.rosInterface.gameState

		teamInfo = filter( lambda x : x[ 'team_number' ] == self.teamNumber, gameState[ 'teams' ] )
		robotInfo = teamInfo[ 0 ][ 'players' ][ self.playerNumber - 1 ]

		if robotInfo[ "secs_till_unpenalized" ] != 0:
			self.ChangeSubBrain( "PenaltyState" )
			return
		
		if robotInfo[ "number_of_red_cards" ] != 0:
			self.ChangeSubBrain( "PenaltyState" )
			return

		if gameState[ "game_state" ] == "STATE_INITIAL":
			self.ChangeSubBrain( "InitialState" )
		elif gameState[ "game_state" ] == "STATE_READY":
			self.ChangeSubBrain( "ReadyState" )
			 #  Enter to the field
		elif gameState[ "game_state" ] == "STATE_SET":
			self.ChangeSubBrain( "SetState" )
		elif gameState[ "game_state" ] == "STATE_PLAYING":
			self.ChangeSubBrain( "PlayState" )
		elif gameState[ "game_state" ] == "STATE_FINISHED":
			self.ChangeSubBrain( "FinishState" ) 


initialState = InitialState()
readyState = ReadyState()
setState = SetState()
playState = PlayState()
finishState = FinishState()
penaltyState = PernaltyState()


main_brain = Controller( initialState = initialState,
						 readyState = readyState,
						 setState = setState,
						 playState = playState,
						 finishState = finishState,
						 penaltyState = penaltyState
 )


