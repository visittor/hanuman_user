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
class ForwardToBall( FSMBrainState ):

	def __init__( self ):

		#	set name
		super( ForwardToBall, self ).__init__( "ForwardToBall" )

	def firstStep( self ):

		self.rosInterface.LocoCommand(	velX = 0.3,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False )

	def step( self ):
		
		rospy.logdebug( "Robot is going forward!" )

	# def leaveStateCallBack( self ):
	# 	self.rosInterface.LocoCommand(	velX = 0.0,
	# 									velY = 0.0,
	# 									omgZ = 0.0,
	# 									commandType = 0,
	# 									ignorable = False ) 

	# 	rospy.logdebug( "Exit state forward!" )

class TurnRighToBall( FSMBrainState ):

	def __init__( self ):
		
		#	Set the name
		super( TurnRighToBall, self ).__init__( "TurnRightToBall" )

	def firstStep( self ):

		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = -0.1,
										commandType = 0,
										ignorable = False )

	def step( self ):
		
		rospy.logdebug( "Robot is turn right!" )

	# def leaveStateCallBack( self ):
	# 	self.rosInterface.LocoCommand(	velX = 0.0,
	# 									velY = 0.0,
	# 									omgZ = 0.0,
	# 									commandType = 0,
	# 									ignorable = False ) 

	# 	rospy.logdebug( "Exit state turn right!" )

class TurnLeftToBall( FSMBrainState ):

	def __init__( self ):
		
		#	Set the name
		super( TurnLeftToBall, self ).__init__( "TurnLeftToBall" )

	def firstStep( self ):

		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.1,
										commandType = 0,
										ignorable = False )

	def step( self ):
		
		rospy.logdebug( "Robot is turn left!" )

	# def leaveStateCallBack( self ):
	# 	self.rosInterface.LocoCommand(	velX = 0.0,
	# 									velY = 0.0,
	# 									omgZ = 0.0,
	# 									commandType = 0,
	# 									ignorable = False )

	# 	rospy.logdebug( "Exit state turn left!" ) 

class StandStill( FSMBrainState ):

	def __init__( self ):
		
		#	Set the name
		super( StandStill, self ).__init__( "StandStill" )

	def firstStep( self ):

		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False )

	def step( self ):
		
		rospy.logdebug( "Robot is standstill" )

class RightKick( FSMBrainState ):

	def __init__( self ):

		#	Set the name
		super( RightKick, self ).__init__( "RightKick" )

	def firstStep( self ):

		self.rosInterface.LocoCommand( command = "RightKick",
									   commandType = 1,
									   ignorable = False )

	def step( self ):
		
		rospy.logdebug( "Right kick like messi!" )

	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False ) 
		rospy.logdebug( "Exit state right kick!" ) 

class LeftKick( FSMBrainState ):

	def __init__( self ):

		#	Set the name
		super( LeftKick, self ).__init__( "LeftKick" )

	def firstStep( self ):

		self.rosInterface.LocoCommand( command = "LeftKick",
									   commandType = 1,
									   ignorable = False )

	def step( self ):
		
		rospy.logdebug( "Left kick like Ronaldoooo!" )

	def leaveStateCallBack( self ):
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False ) 

		rospy.logdebug( "Exit state left kick!" ) 