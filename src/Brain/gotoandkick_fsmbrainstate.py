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

from gotoandkick_sub_brain.followball_brainstate import FollowBall
from gotoandkick_sub_brain.kicking_brainstate import KickTheBall

from gotoandkick_sub_brain.alignball_brainstate import RotateToTheBall
from gotoandkick_sub_brain.findball_brainstate import FindBall
from gotoandkick_sub_brain.slidecurve_brainstate import SlideCurve

from New.scan_goal import ScanGoal
from New.project_mobile_robot import ScanPole, ScanField


from newbie_hanuman.msg import postDictMsg

import numpy as np
import math

import time
import rospy

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

class MainBrain( FSMBrainState ):
	
	def __init__( self ):

		super( MainBrain, self ).__init__( "MainBrain" )

		self.addSubBrain( FindBall( nextState = "ScanField" ) )

		self.addSubBrain( ScanField( nextSubbrain="RotateToTheBall", failStateSubbrain="FindBall" ) )

		self.addSubBrain( RotateToTheBall( previousState = "FindBall", nextState = "FollowBall" ) )
		self.addSubBrain( FollowBall( previousState = "RotateToTheBall", nextState = "ScanPole", findBallState = "FindBall" ) )
		
		# self.addSubBrain( ScanGoal( nextSubbrain = 'SlideCurve', kickingState="KickTheBall", time = 20 ) )
		self.addSubBrain( ScanPole( nextSubbrain = 'SlideCurve', kickingState="KickTheBall", time = 20 ) )
		
		self.addSubBrain( SlideCurve( nextState = "KickTheBall" ) )
		self.addSubBrain( KickTheBall( nextState="FindBall", previousState="FollowBall" ) )

		self.setFirstSubBrain( "FindBall" )

		self.setGlobalVariable( 'PoleColor', ('yellow', 'magenta') )

	def end( self ):
		
		#	terminate pantilt
		self.rosInterface.Pantilt( command = 3 )
			
		#	stop
		self.rosInterface.LocoCommand( velX = 0.0,
					     			   velY = 0.0,
					      			   omgZ = 0.0,
					       			   commandType = 0,
					       			   ignorable = False )

main_brain = MainBrain()
										  