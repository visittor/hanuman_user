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

class KickInMind( FSMBrainState ):

	def __init__( self, nextState = None ):

		#   set name
		super( KickInMind, self ).__init__( "KickInMind" )

		#   get next state
		self.nextState = nextState

	def firstStep( self ):
		
		rospy.loginfo( "Stop because robot cannot kick HAHAHAHAHAHA, fix model and how to detect ball first." )
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False ) 

	def step( self ):

		#	check ball
		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.nextState )

		#	get theta error from vision manger
		ballPolarArray = self.rosInterface.visionManager.ball_polar
		distanceBallToRobot = ballPolarArray[ 0 ]

		#   check distance 
		if distanceBallToRobot >= 0.35:

			self.SignalChangeSubBrain( self.nextState )
 
	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False ) 
										

class KickTheBall( FSMBrainState ):
	
	
	def __init__( self, nextState = None, previousState = None ):
	
		#	set name
		super( KickTheBall, self ).__init__( "KickTheBall" )
 		
		#	get next state
		self.nextState = nextState
		self.previousState = previousState
		
		#	attribute for stored previous time
		self.previousTime = None
		
		#	intial ball side
		self.ballSIde = None
	
	def firstStep( self ):
		
		rospy.logdebug( "Enter to kicking state" )
		
		#	back to findball when lose the ball
		if self.rosInterface.visionManager.ball_confidence == False :
			
			self.SignalChangeSubBrain( self.previousState )
		
		self.previousTime = time.time()
		
		#	get position from y-axis
		yMagnitude = self.rosInterface.visionManager.ball_cart[ 1 ]
		
		#	set side to align
		self.ballSide = 1 if yMagnitude > 0 else -1
	
		self.rosInterface.LocoCommand(	velX = 0.2,
						velY = 0,
						omgZ = 0,
						commandType = 0,
						ignorable = False )
		
	def step( self ):
	
		currentTime = time.time()
		
		rospy.logdebug( "time : {}".format( currentTime - self.previousTime ) )
		
		if currentTime - self.previousTime >= 3.0:
			
			if self.ballSide == 1:
				
				self.rosInterface.LocoCommand( command = "LeftKick",
					      		       commandType = 1,
				  	       		       ignorable = False )
			
			else:
				self.rosInterface.LocoCommand( command = "RightKick",
					      		       commandType = 1,
				  	       		       ignorable = False )
			
			self.rosInterface.Pantilt( command = 3 )
			
			#	set pan tilt motor to initial position
			self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
							position=[ 0, 0 ],
							command=0 )
							
			print "HAHA"
			
			#	stop action
			self.rosInterface.LocoCommand(	velX = 0.0,
							velY = 0,
							omgZ = 0,
							commandType = 0,
							ignorable = False )
			
			time.sleep( 2.5 )
			
			self.SignalChangeSubBrain( self.nextState )
			
					       
					   		
	
		
		
