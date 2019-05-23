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

DistanceThreshold = 0.30

########################################################
#
#	EXCEPTION DEFINITIONS
#

########################################################
#
#	HELPER FUNCTIONS
#

########################################

#
#	CLASS DEFINITIONS
#

class FindBall( FSMBrainState ):
	
	def __init__( self, nextState = "Non

		
		super( FindBall, self ).__init__

 		
		self.nextState = nextState
	
		#	Initial attribute for storin

		self.previousTime = None

		#	Intial attribute for time ou

		self.findBallStateTimeOut = None


		#	Add sub brain
		# self.addSubBrain( WalkingTimeOut( findball="PantiltScan" ) )
		self.addSubBrain( PantiltScan(  ) )

		self.setFirstSubBrain( "PantiltScan" )

	def initialize( self ):

		#	Get time out from config
		self.findBallStateTimeOut = float( self.config[ 'ChangeStateParameter' ][ 'FindballStateTimeOut' ] )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		
		# #	Call pattern
		# self.rosInterface.Pantilt( command = 1, pattern = 'basic_pattern' )
		
		while len( self.rosInterface.visionManager.object_name ) == 0: 
			pass

		#	Get time for first step
		# self.previousTime = time.time()		

	def step( self ):

		#	Get current vision msg
		visionMsg = self.rosInterface.visionManager

		#	Get index
		idxBallObj = visionMsg.object_name.index( 'ball' )

		if visionMsg.object_confidence[ idxBallObj ] >= 0.5:

			rospy.loginfo( "Found ball!!!!" )

			self.rosInterface.LocoCommand(	velX = 0.0,
											velY = 0.0,
											omgZ = 0.0,
											commandType = 0,
											ignorable = False )
			
			self.rosInterface.Pantilt( command = 3 )
			
			self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
						
			self.SignalChangeSubBrain( self.nextState )

		# #	Get current vision msg
		# visionMsg = self.rosInterface.visionManager

		# #	Get index
		# idxBallObj = visionMsg.object_name.index( 'ball' )

		#	Calculate current time
		# currentTime = time.time() - self.previousTime

		# rospy.loginfo( "At step in FindBall. Time >>> {}".format( currentTime ) )

		# if currentTime >= self.findBallStateTimeOut:
		# 	self.ChangeSubBrain( "WalkingTimeOut" )

			# self.SignalChangeSubBrain( self.nextState )

		# if visionMsg.object_confidence[ idxBallObj ] >= 0.5:
			
		# 	self.rosInterface.Pantilt( command = 3 )
			
		# 	self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
						
		# 	self.SignalChangeSubBrain( self.nextState )

class PantiltScan( FSMBrainState ):
	'''	PantiltScan class
	'''

	def __init__( self, walking = "None" ):

		super( PantiltScan, self ).__init__( "PantiltScan" )

		self.nextState = walking

		self.previousTime = None

		self.findBallStateTimeOut = None

	def initialize( self ):
		
		#	Get timeout from config
		self.findBallStateTimeOut = float( self.config[ 'ChangeStateParameter' ][ 'FindballStateTimeOut' ] )

	def firstStep( self ):

		rospy.loginfo( "Enter {} brainstate".format( self.name ) )

		#	Call pantilt scanner
		self.rosInterface.Pantilt( command = 1, pattern = 'basic_pattern' )

		self.previousTime = time.time()

	def step( self ):

		#	Calculate current time
		currentTime = time.time() - self.previousTime

		rospy.loginfo( "At step in PantiltScan. Time >>> {}".format( currentTime ) )

		# if currentTime >= self.findBallStateTimeOut:
		# 	self.SignalChangeSubBrain( self.nextState )

class WalkingTimeOut( FSMBrainState ):
	'''	WalkingTimeOut class
		NOTE:
			When time is out, this state will raise.
			It move robot backward or forward depend on previousDistace as global variable
			Add this class to findball state
	'''
	def __init__( self, findball = "None" ):

		super( WalkingTimeOut, self ).__init__( "WalkingTimeOut" )
		
		self.nextState = findball

		self.previousTime = None

		self.walkingStateTimeOut = None

	def initialize( self ):
		
		#	Get timeout from config
		self.walkingStateTimeOut = float( self.config[ "ChangeStateParameter" ][ "WalkingStateTimeOut" ] )

	def firstStep( self ):

		#	Terminate scan first and not use vision manager in this state
		#	Check previousDistance
		#	If less than 0.5, move backward
		#	Else, move forward
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )

		self.rosInterface.Pantilt( command = 3 )

		#	Set pantilt to zero position
		self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
									position=[ 0, 0 ],
									command=0 )

		#	Get previous distance from global variable
		previousDistance = self.getGlobalVariable( 'previousDistance' )

		if previousDistance >= DistanceThreshold:
			self.rosInterface.LocoCommand(	velX = 0.5,
											velY = 0.0,
											omgZ = 0.0,
											commandType = 0,
											ignorable = False )
		else:
			self.rosInterface.LocoCommand(	velX = -0.5,
											velY = 0.0,
											omgZ = 0.0,
											commandType = 0,
											ignorable = False )

		self.previousTime = time.time()
	
	def step( self ):

		#	Delay it and change back to find ball

		currentTime = time.time() - self.previousTime

		rospy.loginfo( "At step in WalkingTimeOut. Time >>> {}".format( currentTime ) )

		if currentTime >= self.walkingStateTimeOut:

			self.rosInterface.LocoCommand(	velX = 0.0,
											velY = 0.0,
											omgZ = 0.0,
											commandType = 0,
											ignorable = False )
											
			self.SignalChangeSubBrain( self.nextState )						

# main_brain = FindBall()
