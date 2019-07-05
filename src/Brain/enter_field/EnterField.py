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

class _IDLE( FSMBrainState ):

	def __init__( self ):
		
		super( _IDLE, self ).__init__( "_IDLE" )
 
	def firstStep( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = 0.0,
									   commandType = 0,
									   ignorable = False )

	def leaveStateCallBack( self ):
		# self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
		# 					position=[ 0.0, 0.0 ],
		# 					command=0,
		# 					velocity=[100, 100] )
		pass

class _GoForward( FSMBrainState ):

	def __init__( self, duration, nextState = 'None' ):

		super( _GoForward, self ).__init__( "_GoForward" )

		self.duration = duration
		self.nextState = nextState

	def firstStep( self ):
		self.rosInterface.LocoCommand( velX = 0.8,
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

class _TurnLeft( FSMBrainState ):

	def __init__( self ):
		
		super( _TurnLeft, self ).__init__( "_TurnLeft" )

	def firstStep( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = 0.5,
									   commandType = 0,
									   ignorable = False )

	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = 0.0,
									   commandType = 0,
									   ignorable = False )

class _TurnRight( FSMBrainState ):

	def __init__( self ):
		super( _TurnRight, self ).__init__( "_TurnRight" )

	def firstStep( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = -0.5,
									   commandType = 0,
									   ignorable = False )

	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = 0.0,
									   commandType = 0,
									   ignorable = False )

class GoToField( FSMBrainState ):

	def __init__( self, nextState = 'None' ):

		super( GoToField, self ).__init__( "GoToField" )

		self.addSubBrain( _IDLE( ), 'IDLE' )
		self.addSubBrain( _GoForward( 10 ), 'Forward' )
		self.addSubBrain( _TurnRight(), 'TurnRight' )
		self.addSubBrain( _TurnLeft(), 'TurnLeft' )

		self.setFirstSubBrain( 'IDLE' )

		self.nextState = nextState

	def initialize( self ):
		pass

	def firstStep( self ):
		self.startPos = self.getGlobalVariable( 'currRobotPos' )
		self.startTime = time.time( )

	def step( self ):

		currPos = self.getGlobalVariable( 'currRobotPos' )

		if math.fabs(currPos[1]) < 0.30 or self.startPos[1] * currPos[1] < 0:
			self.SignalChangeSubBrain( self.nextState )

		elif self.currSubBrainName == 'Forward':
			return

		elif self.currSubBrainName == 'None':
			self.startTime = time.time( )
			self.ChangeSubBrain( 'IDLE' )

		elif time.time() - self.startTime < 2 and self.prevSubBrainName == 'None':
			return

		elif currPos[1] < 0:
			if math.radians( 270.0 ) <= currPos[2] or currPos[2] < math.radians( 60.0 ): 
				self.ChangeSubBrain( 'TurnLeft' )
			
			elif math.radians( 120.0 ) < currPos[2] < math.radians( 270.0 ):
				self.ChangeSubBrain( 'TurnRight')

			else:
				self.ChangeSubBrain( 'Forward' )
				self.startTime = time.time()

		elif currPos[1] > 0:
			if math.radians( 90.0 ) <= currPos[2] < math.radians( 240.0 ):
				self.ChangeSubBrain( 'TurnLeft' )
			
			elif math.radians( 300.0 ) < currPos[2] or currPos[2] < math.radians( 90.0 ):
				self.ChangeSubBrain( 'TurnRight')

			else:
				self.ChangeSubBrain( 'Forward' )
				self.startTime = time.time()

class TurnToGoal( FSMBrainState ):

	def __init__( self, nextState = 'None' ):

		super( TurnToGoal, self ).__init__( "TurnToGoal" )

		self.addSubBrain( _IDLE( ), 'IDLE' )
		self.addSubBrain( _TurnRight( ), 'TurnRight' )
		self.addSubBrain( _TurnLeft( ), 'TurnLeft' )

		self.setFirstSubBrain( 'IDLE' )

		self.nextState = nextState

	def initialize( self ):
		pass

	def firstStep( self ):
		rospy.loginfo( "Turn")
		self.startTime = time.time()

	def step( self ):

		currPos = self.getGlobalVariable( 'currRobotPos' )

		if time.time() - self.startTime < 3:
			return

		if currPos[2] < math.radians( 15 ) or currPos[2] > math.radians( 345 ):
			self.ChangeSubBrain( 'IDLE' )
			self.startTime = time.time()

		elif currPos[2] <= math.radians( 180 ):

			self.ChangeSubBrain( 'TurnRight' )

		elif currPos[2] > math.radians( 180 ):

			self.ChangeSubBrain( 'TurnLeft' )

	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = 0.0,
									   commandType = 0,
									   ignorable = False )

class EnterField( FSMBrainState ):

	def __init__( self ):

		super( EnterField, self ).__init__( "EnterField" )

		self.addSubBrain( _IDLE(), 'IDLE' )
		self.addSubBrain( GoToField('TurnToGoal') )
		self.addSubBrain( TurnToGoal('None') )

		self.setFirstSubBrain( 'IDLE' )

	def initialize( self ):

		if self.config.has_key('Localization'):
			self.x_init = float( self.config['Localization'].get( 'init_x', 0.0 ) )
			self.y_init = float( self.config['Localization'].get( 'init_y', 0.0 ) )
			self.w_init = float( self.config['Localization'].get( 'init_w', 0.0 ) )

		else:
			self.x_init = 0.0
			self.y_init = 0.0
			self.w_init = 0.0

	def firstStep( self ):

		rospy.loginfo( "Enter to the {} brain state".format( self.name ) )

		self.rosInterface.LocoCommand( command = 'sitToStand', commandType = 1 )

		time.sleep( 3 )

		self.rosInterface.localization_command( command = 1, x = self.x_init,
												y = self.y_init, w = self.w_init )

		self.startTime = time.time( )

		currPos = self.rosInterface.robot_pose
		currPos.theta %= 2*np.pi

		self.setGlobalVariable( 'currRobotPos', (currPos.x, currPos.y, currPos.theta) )

	def step( self ):

		currPos = self.rosInterface.robot_pose
		currPos.theta %= 2*np.pi

		self.setGlobalVariable( 'currRobotPos', (currPos.x, currPos.y, currPos.theta) )

		if self.currSubBrainName == 'IDLE' and time.time() - self.startTime > 5:
			self.ChangeSubBrain( 'GoToField' )

class ReadyState( FSMBrainState ):

	def __init__( self ):

		super( ReadyState, self ).__init__( "ReadyState" )

		self.addSubBrain( EnterField( ) )

		self.setFirstSubBrain( 'None' )

		self.firstTime = True

	def initial( self ):

		pass

	def firstStep( self ):

		self.rosInterface.Pantilt( command = 1, pattern = "basic_pattern" )
		
		#	Add sit to stand
		# self.rosInterface.LocoCommand( command = 'sitToStand', commandType = 1 )

		rospy.loginfo( "Robot is going to half of field" )


	def step( self ):

		if self.firstTime:
			self.ChangeSubBrain( 'EnterField' )
			self.firstTime = False

main_brain = ReadyState()