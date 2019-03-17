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

from newbie_hanuman.msg import postDictMsg

import numpy as np
import math

import time
import rospy

########################################################
#
#	GLOBALS
#

DefaultLoopTimeToLookAtObject = 1.0
DefaultObject = 'ball'
DefaultPantiltPattern = 'basic_pattern'

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
	
	def __init__( self, nextState = None ):
		
		super( FindBall, self ).__init__( 'FindBall' )
		
		#	initial object
		self.objectIndex = None
		
		#	set state
		self.nextState = nextState
		
	def firstStep( self ):
		
		rospy.loginfo( "Enter find ball state" )
		
		self.rosInterface.LocoCommand(	velX = 0.0,
						velY = 0.0,
						omgZ = 0.0,
						commandType = 0,
						ignorable = False )
		
		#	loop for wait msg
		while len( self.rosInterface.visionManager.object_name ) == 0: 
			pass
		
		self.rosInterface.Pantilt( command = 1, pattern = DefaultPantiltPattern )
		
		#	get index object
		self.objectIndex = self.rosInterface.visionManager.object_name.index( DefaultObject )
		
	def step( self ):
		
		if self.rosInterface.visionManager.object_confidence[ self.objectIndex ] >= 0.5:
			
			self.rosInterface.Pantilt( command = 3 )
			
			self.SignalChangeSubBrain( self.nextState )

class TrackingBall( FSMBrainState ):


	def __init__( self, previousState = None, nextState = None ):

		super( TrackingBall, self ).__init__( 'TrackingBall' )
		
		#	set state
		self.nextState = nextState
		self.previousState = previousState
		
		#	initial num frame for check it's the ball
		self.numFrame = 0
		
		#	initial object
		self.objectIndex = None
		
		#	time stamp
		self.stampTime = 0
		
		#	flag rotate
		self.neverRotateFlag = True
		
	def firstStep( self ):
		'''
			first step before execute this brain
		'''

		rospy.loginfo( "Enter tracking ball state" )

		#	re-initial num frame for check it's the ball
		self.numFrame = 0
		
		#	re-initial flag rotate
		self.neverRotateFlag = True
		
		while len( self.rosInterface.visionManager.object_name ) == 0: 
			pass
		
		#	get index object
		self.objectIndex = self.rosInterface.visionManager.object_name.index( DefaultObject )
		
		#	stamp time before enter step		
		self.stampTime = time.time()
		
		
	def step( self ):
		
		#	get current time
		currentStepTime = time.time()
		
		#	time remain 
		timeRemain = currentStepTime - self.stampTime
		
		if self.rosInterface.visionManager.object_confidence[ self.objectIndex ] >= 0.5:
		
			rospy.logdebug( "Found ball" )
		
			if timeRemain >= DefaultLoopTimeToLookAtObject:
				
				#	STARE!!!
				self.rosInterface.Pantilt( command = 2, pattern = DefaultObject )
				
				#	get theta ref to ball
				thetaWrtRobotDegree = self.rosInterface.visionManager.pos2D_polar[ self.objectIndex ].y
				
				#	check angle if not exceed 10 degree move forward
				if abs( thetaWrtRobotDegree ) <= math.radians( 10 ):

					self.rosInterface.LocoCommand(	velX = 0.0,
									velY = 0.0,
									omgZ = 0.0,
									commandType = 0,
									ignorable = False )
					
					self.SignalChangeSubBrain( self.nextState )
				
				if self.neverRotateFlag:
								
					#	get sign
					sign = abs( thetaWrtRobotDegree ) / thetaWrtRobotDegree

					self.rosInterface.LocoCommand(	velX = 0.0,
									velY = 0.0,
									omgZ = sign * 0.15,
									commandType = 0,
									ignorable = False )
									
					self.neverRotateFlag = False
				
				#	echo theta wrt robot
				rospy.logdebug( "theta : {}".format( thetaWrtRobotDegree ) )
				
				#	Stamp current time
				self.stampTime = time.time()
					
			if self.numFrame < 10:	
				self.numFrame += 1
					
		else:
			
			rospy.logdebug( "Lost" )
			
			#	decrease num frame
			self.numFrame -= 1
			
			#	reset flag
			self.neverRotateFlag = True
			
			if self.numFrame < 0:
				self.SignalChangeSubBrain( self.previousState )
				
		rospy.logdebug( "num frame when see the ball : {}".format( self.numFrame ) )
