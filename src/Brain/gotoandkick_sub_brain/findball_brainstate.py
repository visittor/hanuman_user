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

class TurnLeftTimer( FSMBrainState ):

	def __init__( self, time, nextState = 'None' ):

		super( TurnLeftTimer, self ).__init__( 'TurnLeftTimer' )

		self.duration = time
		self.nextState = nextState

	def firstStep( self ):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.5,
										commandType = 0,
										ignorable = False )
		self.startTime = time.time( )

	def step( self ):
		if time.time( ) - self.startTime > self.duration:
			self.SignalChangeSubBrain( self.nextState )

	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = 0.0,
									   commandType = 0,
									   ignorable = False )		
class TurnRightTimer( FSMBrainState ):

	def __init__( self, time, nextState = 'None' ):

		super( TurnRightTimer, self ).__init__( 'TurnRightTimer' )

		self.duration = time
		self.nextState = nextState

	def firstStep( self ):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = -0.5,
										commandType = 0,
										ignorable = False )
		self.startTime = time.time( )

	def step( self ):
		if time.time( ) - self.startTime > self.duration:
			self.SignalChangeSubBrain( self.nextState )

	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = 0.0,
									   commandType = 0,
									   ignorable = False )	

class FindBall( FSMBrainState ):
	
	def __init__( self, nextState = "None" ):

		super( FindBall, self ).__init__( "FindBall" )

		self.turnDuration = 2

		self.addSubBrain( TurnLeftTimer( self.turnDuration ) )
		self.addSubBrain( TurnRightTimer( self.turnDuration ) )

		self.setFirstSubBrain( 'None' )

		self.findBallStateTimeOut = None

		self.stepDirection = [ 1, -1, -1, 1 ]
		self.stepIndex = 0

	def initialize( self ):

		#	Get time out from config
		self.findBallStateTimeOut = float( self.config[ 'ChangeStateParameter' ][ 'FindballStateTimeOut' ] )
		self.findBallStateTimeOut += self.turnDuration

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		self.stepIndex = 0
		self.timeStart = time.time( )

	def step( self ):

		if time.time( ) - self.timeStart > self.findBallStateTimeOut:

			direction = self.stepDirection[ self.stepIndex ]

			if direction > 0:
				self.ChangeSubBrain( 'TurnLeftTimer' )
			else:
				self.ChangeSubBrain( 'TurnRightTimer' )

			self.stepIndex = ( self.stepIndex + 1 ) % len( self.stepDirection )
			self.timeStart = time.time( )

	def leaveStateCallBack( self ):

		self.stopRobotBehavior( )
		
	def stopRobotBehavior( self ):
		#	stop
		self.rosInterface.LocoCommand( velX = 0.0,
					       			   velY = 0.0,
					       			   omgZ = 0.0,
					       			   commandType = 0,
					       			   ignorable = False )		
		# self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
		# 							   ignorable =  False )