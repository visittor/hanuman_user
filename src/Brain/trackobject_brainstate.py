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

import numpy as np
import rospy

import time

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

class TrackObject( FSMBrainState ):

	def __init__( self ):

		super( TrackObject, self ).__init__( "TrackObject" )

		self.timeStamp = time.time()
		self.lookatOrange = True
		self.lookatMagenta = False

	def firstStep( self ):

		rospy.loginfo( "Enter to {} brainstate".format( self.name ) )

	def step( self ):

		if time.time() - self.timeStamp > 2: 

			if self.lookatOrange:
				rospy.loginfo( "Look at magenta : {}, orange {}".format( self.lookatMagenta, self.lookatOrange ) )
				self.rosInterface.Pantilt( command = 2, pattern = 'ball_orange' )
				self.lookatOrange = False
				self.lookatMagenta = True
			
			elif self.lookatMagenta:
				rospy.loginfo( "Look at magenta : {}, orange {}".format( self.lookatMagenta, self.lookatOrange ) )
				self.rosInterface.Pantilt( command = 2, pattern = 'ball_magenta' )
				self.lookatOrange = True
				self.lookatMagenta = False

			self.timeStamp = time.time()
		
main_brain = TrackObject()