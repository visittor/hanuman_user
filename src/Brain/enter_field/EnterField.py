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

from default_config import getParameters

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
		pass

	def firstStep( self ):
		self.rosInterface.LocoCommand( velX = 0.0,
									   velY = 0.0,
									   omgZ = 0.0,
									   commandType = 0,
									   ignorable = False )

		self.rosInterface.Pantilt( command = 1, pattern = "basic_pattern" )

	def leaveStateCallBack( self ):
		self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
							position=[ 0.0, 0.0 ],
							command=0,
							velocity=[100, 100] )

class _GoForward( FSMBrainState ):

	def __init__( self, duration, nextState = 'None' ):
		self.duration = duration
		self.nextState = nextState

	def firstStep( self ):
		self.rosInterface.LocoCommand( velX = 0.5,
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
		pass

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
		pass

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

		self.addSubBrain( _IDLE( ), 'IDLE' )
		self.addSubBrain( _GoForward( 5 ), 'Forward' )
		self.addSubBrain( _TurnRight(), 'TurnRight' )
		self.addSubBrain( _TurnLeft(), 'TurnLeft' )

		self.setFirstSubBrain( '_IDLE' )

		self.nextState = nextState

	def initialize( self ):
		pass

	def firstStep( self ):
		self.startPos = self.getGlobalVariable( 'currRobotPos' )

	def step( self ):

		currPos = self.getGlobalVariable( 'currRobotPos' )

		if self.currSubBrainName == 'Forward':
			return

		if math.fabs(currPos[1]) < 0.75 or self.startPos[1] * currPos[1] < 0:
			self.SignalChangeSubBrain( self.nextState )

		elif currPos[1] < 0:
			if math.radians( 270.0 ) <= currPos[2] or currPos[2] < math.radians( 60.0 ): 
				self.ChangeSubBrain( 'TurnLeft' )
			
			elif math.radians( 120.0 ) < currPos[2] < math.radians( 270.0 ):
				self.ChangeSubBrain( 'TurnRight')

			else:
				self.ChangeSubBrain( 'Forward' )

		elif currPos[1] > 0:
			if -math.radians( 90.0 ) <= currPos[2] < math.radians( 60.0 ): 
				self.ChangeSubBrain( 'TurnRight' )
			
			elif math.radians( 120.0 ) < currPos[2] < math.radians( 270.0 ):
				self.ChangeSubBrain( 'TurnLeft')

			else:
				self.ChangeSubBrain( 'Forward' )

class TurnToGoal( FSMBrainState ):

	def __init__( self, nextState = 'None' ):

		self.addSubBrain( _IDLE( ), 'IDLE' )
		self.addSubBrain( _TurnRight( ), 'TurnRight' )
		self.addSubBrain( _TurnLeft( ), 'TurnLeft' )

		self.setFirstSubBrain( 'IDLE' )

		self.nextState = nextState

	def initialize( self ):
		pass

	def firstStep( self ):
		pass

	def step( self ):

		currPos = self.getGlobalVariable( 'currRobotPos' )

		if currPos[2] < math.radians( 30 ) or currPos[2] > math.radians( 330 ):
			self.SignalChangeSubBrain( self.nextState )

		if currPos[2] <= math.radians( 180 ):

			self.ChangeSubBrain( 'TurnRight' )

		elif currPos[2] > math.radians( 180 ):

			self.ChangeSubBrain( 'TurnLeft' )

class EnterField( FSMBrainState ):

	def __init__( self ):

		self.addSubBrain( _IDLE(), 'IDLE' )
		self.addSubBrain( GoToField('TurnToGoal') )
		self.addSubBrain( TurnToGoal('None') )

		self.setFirstSubBrain( '_IDLE' )

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

		self.rosInterface.LocoCommand( command = 'sitToStand', commandType = 1 )

		time.sleep( 3 )

		self.rosInterface.localization_command( command = 1, x = self.x_init,
												y = self.y_init, w = self.w_init )

		self.startTime = time.time( )

		currPos = self.rosInterface.robot_pose
		currPos.w %= 2*np.pi

		self.setGlobalVariable( 'currRobotPos', (currPos.x, currPos.y, currPos.w) )

	def step( self ):

		currPos = self.rosInterface.robot_pose
		currPos.w %= 2*np.pi

		self.setGlobalVariable( 'currRobotPos', (currPos.x, currPos.y, currPos.w) )

		if self.currSubBrainName == 'IDLE' and time.time() - self.startTime > 3:
			self.ChangeSubBrain( 'GoToField' )

class ReadyState( FSMBrainState ):

	def __init__( self ):

		self.addSubBrain( EnterField( ) )

		self.setFirstSubBrain( 'None' )

		self.firstTime = True

	def initial( self ):

		pass

	def firstStep( self ):
		pass

	def step( self ):

		if self.firstTime:
			self.ChangeSubBrain( 'EnterField' )
			self.firstTime = False
