#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
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
import math

import time
import rospy

from findball_brainstate import FindBall
from alignball_brainstate import RotateToTheBall
from followball_brainstate import FollowBall
from slidecurve_brainstate import SlideCurve
from kicking_brainstate import KickTheBall

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

class GotoballAndKick( FSMBrainState ):
	
	def __init__( self ):
		
		super( GotoballAndKick, self ).__init__( "GotoballAndKick" )
		
		self.addSubBrain( FindBall( nextState = "RotateToTheBall" ) )
		self.addSubBrain( RotateToTheBall( previousState = "FindBall", nextState = "FollowBall" ) )
		self.addSubBrain( FollowBall( previousState = "RotateToTheBall", nextState = "SlideCurve", findBallState = "FindBall" ) )
		self.addSubBrain( SlideCurve( nextState = "KickTheBall" ) )
		self.addSubBrain( KickTheBall( nextState = "None" ) )
		
		self.setFirstSubBrain( "FindBall" )
		
	def end( self ):
		
		#	terminate pantilt
		self.rosInterFace.Pantilt( command = 3 )
			
		#	stop
		self.rosInterface.LocoCommand( velX = 0.0,
					       velY = 0.0,
					       omgZ = 0.0,
					       commandType = 0,
					       ignorable = False )
main_brain = GotoballAndKick()
