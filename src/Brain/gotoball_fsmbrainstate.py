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

from findball_brainstate import TrackingBall
from gotoball_brainstate import FollowBall
from kicking_brainstate import KickTheBall
from rotatetotheball_brainstate import RotateToTheBall

import numpy as np

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


trackingBall = TrackingBall( nextState = "RotateToTheBall" )

followBall = FollowBall( kickingState = "KickTheBall", 
			 trackingState = "TrackingBall" )

kickXSO = KickTheBall( nextState = "TrackingBall" )   

main_brain = MainBrain( "main_brain" )
main_brain.addSubBrain( trackingBall )
main_brain.addSubBrain( followBall )
main_brain.addSubBrain( kickXSO )
main_brain.setFirstSubBrain( "TrackingBall" )
