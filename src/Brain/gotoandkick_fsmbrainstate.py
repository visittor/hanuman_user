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

from gotoandkick_sub_brain.gotoball_brainstate import GotoBall

from gotoandkick_sub_brain.kicking_brainstate import KickTheBall
from gotoandkick_sub_brain.findball_brainstate import FindBall
from gotoandkick_sub_brain.slidecurve_brainstate import SlideCurve

from New.scan_goal import ScanGoal
from New.project_mobile_robot import ScanPole, ScanField, Idle


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

class _IDLE( FSMBrainState ):

	def __init__( self, nextState = 'None' ):

		super( _IDLE, self ).__init__( '_IDLE' )

		self.nextState = nextState

	def firstStep( self ):

		self.stopRobotBehavior( )
		self._startTime = time.time( )

	def step( self ):

		if time.time( ) - self._startTime > 5:
			self.SignalChangeSubBrain( self.nextState )

	def stopRobotBehavior( self ):
		#	stop	
		self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   ignorable =  False )

class MainBrain( FSMBrainState ):
	
	def __init__( self ):

		super( MainBrain, self ).__init__( "MainBrain" )

		self.addSubBrain( GotoBall( nextState = 'ScanGoal' ) )

		self.addSubBrain( ScanGoal( nextSubbrain = 'SlideCurve', kickingState="KickTheBall", time = 10 ) )
		# self.addSubBrain( ScanPole( nextSubbrain = 'SlideCurve', kickingState="KickTheBall", time = 20 ) )
		
		self.addSubBrain( SlideCurve( nextState = "KickTheBall" ) )
		self.addSubBrain( KickTheBall( nextState="GotoBall", previousState="FollowBall" ) )

		self.setFirstSubBrain( "GotoBall" )

		#	No.3
		# self.setGlobalVariable( 'PoleColor', ('blue', 'orange') )

	def step( self ):
		# rospy.loginfo( "		Running on step mainbrain..." )
		pass
	
	def end( self ):
		
		#	terminate pantilt
		self.rosInterface.Pantilt( command = 3 )
			
		#	stop
		self.rosInterface.LocoCommand( velX = 0.0,
					     			   velY = 0.0,
					      			   omgZ = 0.0,
					       			   commandType = 0,
					       			   ignorable = False )

# main_brain = MainBrain()
										  