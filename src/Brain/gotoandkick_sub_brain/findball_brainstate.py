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

class BackwardTimer( FSMBrainState ):

	def __init__( self, time, nextState = 'None' ):

		super( BackwardTimer, self ).__init__( 'BackwardTimer' )

		self.duration = time
		self.nextState = nextState

	def firstStep( self ):
		self.rosInterface.LocoCommand(	velX = -0.6,
										velY = 0.0,
										omgZ = 0.0,
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

class StopTimer( FSMBrainState ):

	def __init__( self, time, nextState = 'None' ):

		super( StopTimer, self ).__init__( 'StopTimer' )

		self.duration = time
		self.nextState = nextState

	def firstStep( self ):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False )
		self.startTime = time.time( )

	def step( self ):
		if time.time( ) - self.startTime > self.duration:
			self.SignalChangeSubBrain( self.nextState )

class ForwardTimer( FSMBrainState ):

	def __init__( self, time, nextState = 'None' ):

		super( ForwardTimer, self ).__init__( 'ForwardTimer' )

		self.duration = time
		self.nextState = nextState

	def firstStep( self ):
		self.rosInterface.LocoCommand(	velX = 0.5,
										velY = 0.0,
										omgZ = 0.0,
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

		self.duration = 5

		self.addSubBrain( TurnLeftTimer( self.duration ), 'left' )
		self.addSubBrain( TurnRightTimer( self.duration ), 'right' )
		self.addSubBrain( ForwardTimer( self.duration ), 'forward' )
		self.addSubBrain( BackwardTimer( self.duration ), 'backward' )


		self.setFirstSubBrain( 'None' )

		self.findBallStateTimeOut = None

		self.stepDirection = ['stop', 'backward', 'stop', 'left', 
							'stop', 'right', 'right', 'stop', 
							'left']
		self.stepIndex = 0

	def initialize( self ):

		#	Get time out from config
		self.findBallStateTimeOut = float( self.config[ 'ChangeStateParameter' ][ 'FindballStateTimeOut' ] )
		self.findBallStateTimeOut += self.duration

		self.addSubBrain( StopTimer( self.findBallStateTimeOut ), 'stop' )

		if self.config.has_key('ChangeStateParameter'):
			if self.config['ChangeStateParameter'].has_key('FindBallPattern'):
				keys = sorted(self.config['ChangeStateParameter']['FindBallPattern'], key=lambda x:int(x))
				self.stepDirection = []
				for k in keys:
					val = self.config['ChangeStateParameter']['FindBallPattern'][k]
					if val in self.subBrains.keys( ):
						self.stepDirection.append( val )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		self.stepIndex = 0
		self.timeStart = time.time( )

	def step( self ):

		if self.currSubBrainName == 'None':

			direction = self.stepDirection[ self.stepIndex ]

			self.ChangeSubBrain( direction )

			self.stepIndex = ( self.stepIndex + 1 ) % len( self.stepDirection )

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

main_brain = ForwardTimer(10)