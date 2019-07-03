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
from abnormal_brainstate import AbnormalState

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

# history_fn = '/'.join(__file__.split('/')[:-1] + ['history'])
history_fn = os.path.join( os.getenv( 'ROS_WS' ), 'history' )

class Controller( FSMBrainState ):

	def __init__( self, initialState, readyState, setState, playState, finishState, penaltyState, abnormalState,
				enter_field = None ):
		
		super( Controller, self ).__init__( "Controller" )

		self.playerNumber = None
		self.teamNumber = None

		self.prevState = ''

		self.addSubBrain( initialState, "InitialState" )
		self.addSubBrain( readyState, "ReadyState" )
		self.addSubBrain( setState, "SetState" )
		self.addSubBrain( playState, "PlayState" )
		self.addSubBrain( finishState, "FinishState" )

		self.addSubBrain( abnormalState, "AbnormalState" )

		self.addSubBrain( penaltyState, "PenaltyState" )

		self.setFirstSubBrain( "None" )

		if enter_field is not None:
			self.addSubBrain( enter_field, "EnterField" )
		
		self.doEnterfield = False

		self.startEnterField = -1

	def initialize( self ):

		self.teamNumber = int( self.config[ "GameControllerParameter" ][ "Team" ] )
		self.playerNumber = int( self.config[ "GameControllerParameter" ][ "Player" ] )

	def firstStep( self ):
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )

		if os.path.isfile(history_fn):
			with open( history_fn, 'r' ) as f:
				state = f.read( )
			
			if self.subBrains.has_key("EnterField") and state in ["PENALIZED", "STATE_PLAYING"]:
				# self.startEnterField = time.time( )
				# self.ChangeSubBrain( "EnterField" )
				self.doEnterfield = True

	def step( self ):

		if self.startEnterField > 0.0 and time.time() - self.startEnterField<46.0:
			return
		
		gameState = self.rosInterface.gameState

		if gameState == None:
			return

		teamInfo = filter( lambda x : x[ 'team_number' ] == self.teamNumber, gameState[ 'teams' ] )
		robotInfo = teamInfo[ 0 ][ 'players' ][ self.playerNumber - 1 ]

		# print "Second to unpenalize : {}".format( robotInfo[ "secs_till_unpenalized" ] )
		# print "Number of red cards : {}".format( robotInfo[ "number_of_red_cards" ] )

		#	Game controller information
		# rospy.loginfo( "				Listening from gamecontroller..." )
		# rospy.loginfo( "			GameState : {}".format( gameState[ "game_state" ] ) )
		# rospy.loginfo( "			Secs to unpenalize : {}".format( robotInfo[ "secs_till_unpenalized" ] ) )
		# rospy.loginfo( "			Number of red card : {}".format( robotInfo[ "number_of_red_cards" ] ) )

		state = gameState[ "game_state" ]
		secondState = gameState[ "secondary_state" ]

		if robotInfo["secs_till_unpenalized"] != 0:
			state = "PENALIZED"
		elif robotInfo[ "number_of_red_cards" ] != 0:
			state = "PENALIZED"

		if self.prevState != state:
			with open( history_fn, 'w' ) as f:
				f.write( state )

			self.prevState = state

		if state == "PENALIZED":
			self.ChangeSubBrain( "PenaltyState" )
			self.doEnterfield = True
			return

		if gameState[ "secondary_state" ] == "STATE_NORMAL":

			if gameState[ "game_state" ] == "STATE_INITIAL":
				self.ChangeSubBrain( "InitialState" )

			elif gameState[ "game_state" ] == "STATE_READY":
				self.ChangeSubBrain( "ReadyState" )

			elif gameState[ "game_state" ] == "STATE_SET":
				self.ChangeSubBrain( "SetState" )

			elif gameState[ "game_state" ] == "STATE_PLAYING":
				if self.doEnterfield:
					self.ChangeSubBrain( "EnterField" )
					self.doEnterfield = False
					self.startEnterField = time.time()
				else:
					self.ChangeSubBrain( "PlayState" )
			
			elif gameState[ "game_state" ] == "STATE_FINISHED":
				self.ChangeSubBrain( "FinishState" ) 
		
		else:

			self.ChangeSubBrain( "AbnormalState" )	


#	For debug
initialState = InitialState()
readyState = ReadyState()
setState = SetState()
playState = PlayState()
finishState = FinishState()
penaltyState = PernaltyState()
abnormal = AbnormalState()


main_brain = Controller( initialState = initialState,
						 readyState = readyState,
						 setState = setState,
						 playState = playState,
						 finishState = finishState,
						 penaltyState = penaltyState,
						 abnormalState = abnormal,
						 enter_field = EnterField_dummy(),
 )


