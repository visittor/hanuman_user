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
import time
import rospy
import numpy as np

########################################################
#
#	LOCAL IMPORTS
#

from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

from gameController.Receiver import GAME_TYPE, GAME_STATE_TYPE, SECONDARY_STATE_TYPE, TEAM_COLOR_TYPE

from newbie_hanuman.msg import gameState, TeamInfo, RobotInfo

from gameControl_brainstate import InitialState, ReadyState, SetState, PlayState, FinishState

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

class GameState( FSMBrainState ):

    def __init__( self ):
        super( GameState, self ).__init__("GameState")
		#   initial instance state
        initialState = InitialState()
        readyState = ReadyState()
        setState = SetState()
        playState = PlayState()
        finishState = FinishState()
		#   Add sub brain
        self.addSubBrain( initialState )
        self.addSubBrain( readyState )
        self.addSubBrain( setState )
        self.addSubBrain( playState )
        self.addSubBrain( finishState )
        #   set first sub brain
        self.setFirstSubBrain( "InitialState" )
        #   attribute for store game state package
        self.gameStateMsg = None
        #   attribute for manipulation
        self.gameState = None
        self.kickOffTeam = None
        self.teamID = [None,None]

    def getGameStateMsg( self ):
        self.gameStateMsg = self.rosInterface.GameState
        self.__updateAttribute()

    def __updateAttribute(self):
        self.gameState = GAME_STATE_TYPE[ self.gameStateMsg.game_state ]
        self.kickOffTeam = self.gameStateMsg.kick_of_team
        for i in range(2):
            self.teamID[i] = self.gameStateMsg.teams[i].team_number

    def firstStep(self):
        print( "Start Game" )

    def step(self):
        self.getGameStateMsg()

        if self.gameState == 'STATE_INITIAL':
            self.ChangeSubBrain('InitialState')
        elif self.gameState == 'STATE_READY':
            self.ChangeSubBrain('ReadyState')
        elif self.gameState == 'STATE_SET':
            self.ChangeSubBrain('SetState')
        elif self.gameState == 'STATE_PLAYING':
            self.ChangeSubBrain('PlayState')
        elif self.gameState == 'STATE_FINISHED':
            self.ChangeSubBrain('FinishState')
        
            
main_brain = GameState()
			
		





