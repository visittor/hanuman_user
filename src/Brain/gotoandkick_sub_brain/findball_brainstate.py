#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun (NeverHoliday) Hayeeyama
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

class FindBall( FSMBrainState ):
	
	def __init__( self, nextState = "None" ):
		
		super( FindBall, self ).__init__( "FindBall" )
 		
		self.nextState = nextState
	
	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		
		#	Call pattern
		self.rosInterface.Pantilt( command = 1, pattern = 'basic_pattern' )
		
		while len( self.rosInterface.visionManager.object_name ) == 0: 
			pass
		
	def step( self ):
		
		#	Get current vision msg
		visionMsg = self.rosInterface.visionManager
		
		#	Get index
		idxBallObj = visionMsg.object_name.index( 'ball' )
		
		if visionMsg.object_confidence[ idxBallObj ] >= 0.5:
			
			self.rosInterface.Pantilt( command = 3 )
			
			self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
			
			self.SignalChangeSubBrain( self.nextState )
						
#main_brain = FindBall()
