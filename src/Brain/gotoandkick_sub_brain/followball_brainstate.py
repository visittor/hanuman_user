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


class FollowBall( FSMBrainState ):

	def __init__( self, previousState = "None", nextState = "None", findBallState = "None" ):
		
		super( FollowBall, self ).__init__( 'FollowBall' )
		
		self.previousState = previousState
		self.nextState = nextState
		
		self.findBallState = findBallState
		
		self.numFrameNotDetectBall = None
		
	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )	
		
		#	re-initial num frame that not detect ball
		self.numFrameNotDetectBall = 0

		self.rosInterface.Pantilt( command = 3 )
			
		self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
		
	def step( self ):
		
		#	Get current vision msg
		visionMsg = self.rosInterface.visionManager
		
		#	Get index
		idxBallObj = visionMsg.object_name.index( 'ball' )
		
		#	Check confidence if model could detect ball
		if visionMsg.object_confidence[ idxBallObj ] > 0.5:
			
			#	get distance and angle
			distanceWrtBall = visionMsg.pos3D_cart[ idxBallObj ].x
			sideDistanceWrtBall = visionMsg.pos3D_cart[ idxBallObj ].y
			
			thetaWrtBall = visionMsg.pos2D_polar[ idxBallObj ].y
			
			
			if abs( thetaWrtBall ) > math.radians( 12 ):
				
				#	it should switch to first state to find the ball

				self.SignalChangeSubBrain( self.previousState )

			if distanceWrtBall >= 0.20:
				
				#	Get side to kick in kicking brain state
				direction = 1 if sideDistanceWrtBall > 0 else -1
				
				self.setGlobalVariable( 'direction', direction )	
			
				self.rosInterface.LocoCommand( velX = 0.3,
							       			   velY = 0.15,
							       			   omgZ = 0.0,
							       			   commandType = 0,
							       			   ignorable = False )
			else:
	
				self.stopRobotBehavior()

				self.SignalChangeSubBrain( self.nextState )
				
			
		else:
			#	it should switch to first state to find the ball
			self.stopRobotBehavior()
			
			self.SignalChangeSubBrain( self.findBallState )
			
			
	def stopRobotBehavior( self ):
		
		#	terminate pantilt
		self.rosInterface.Pantilt( command = 3 )
			
		#	stop
		self.rosInterface.LocoCommand( velX = 0.0,
					       			   velY = 0.0,
					       			   omgZ = 0.0,
					       			   commandType = 0,
					       			   ignorable = False )		
