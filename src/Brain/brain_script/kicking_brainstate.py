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

import time
import rospy

import math

########################################################
#
#	GLOBALS
#

DefaultObject = 'ball'

DefaultDistanceToKicking = 0.10

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
										

class KickTheBall( FSMBrainState ):
	
	
	def __init__( self, nextState = None, previousState = None ):
	
		#	set name
		super( KickTheBall, self ).__init__( "KickTheBall" )
 		
		#	get next state
		self.nextState = nextState
		self.previousState = previousState
	
		#	initial object index
		self.objectIndex = None
			
	def firstStep( self ):

		rospy.loginfo( "Enter kick the ball state" )
	
		self.objectIndex = self.rosInterface.visionManager.object_name.index( DefaultObject )

#		if self.rosInterface.visionManager.object_confidence[ self.objectIndex ] < 0.5:
#			self.SignalChangeSubBrain( self.nextState )
				
	def step( self ):
		
		visionMsg = self.rosInterface.visionManager
		
		#	check side
		#	get position from y-axis
		yMagnitude = visionMsg.pos3D_cart[ self.objectIndex ].y
		xMagnitude = visionMsg.pos3D_cart[ self.objectIndex ].x
		
		print "x mag {}".format( xMagnitude )
		print "y mag {}".format( yMagnitude )
		
		if xMagnitude <= DefaultDistanceToKicking :
			#	select kick
			if yMagnitude > 0:
				self.rosInterface.LocoCommand( command = "LeftKick", commandType = 1 )
			else:
				self.rosInterface.LocoCommand( command = "RightKick", commandType = 1 )
			
			#	delay after kick wait for behaviour while robot kick
			time.sleep( 3 )
		
			self.SignalChangeSubBrain( self.nextState )
			
#		for _ in range( int( math.ceil(xMagnitude * 10) ) ): 
#		#	one step
#			self.rosInterface.LocoCommand(	velX = 0.8,
#							velY = 0.0,
#							omgZ = 0.0,
#							command = 'OneStepWalk' )
#
#			time.sleep( 3 )

		if abs(yMagnitude) > 0.02:
			sign = 1 if yMagnitude > 0 else -1
			self.rosInterface.LocoCommand( velX = 0.0,
							velY = sign * 0.5,
							omgZ = 0.0,
							command = 'oneStepWalk' )
							
		elif abs(xMagnitude) > 0.1:
			self.rosInterface.LocoCommand(	velX = 0.6,
							velY = 0.4,
							omgZ = 0.0, )

			time.sleep( xMagnitude*10 )
			
			#	select kick
			if yMagnitude > 0:
				self.rosInterface.LocoCommand( command = "LeftKick", commandType = 1 )
			else:
				self.rosInterface.LocoCommand( command = "RightKick", commandType = 1 )

			#	delay after kick wait for behaviour while robot kick
			time.sleep( 4 )

			self.SignalChangeSubBrain( self.nextState )
#		
#		#	select kick
#		if yMagnitude > 0:
#			self.rosInterface.LocoCommand( command = "LeftKick", commandType = 1 )
#		else:
#			self.rosInterface.LocoCommand( command = "RightKick", commandType = 1 )
#			
#		#	delay after kick wait for behaviour while robot kick
#		time.sleep( 3 )
#		
#		self.SignalChangeSubBrain( self.nextState )
					   		
