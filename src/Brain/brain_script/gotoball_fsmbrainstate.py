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

from findball_brainstate import TrackingBall, FindBall, TrackingBall2
from gotoball_brainstate import FollowBall
from kicking_brainstate import KickTheBall
from rotatetotheball_brainstate import RotateToTheBall
from aligngoal_brainstate import AlignGoal

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

class MainBrain( FSMBrainState ):
	# def firstStep(self):
	# 	self.rosInterface.Pantilt(	pattern="basic_pattern",
	# 								command=1)
	
#	def firstStep( self ):
#		self.rosInterface.LocoCommand( command = 'sitToStand', commandType = 1 )
#		
#		time.sleep( 2 )
	
	def end(self):

		self.rosInterface.Pantilt( command = 3 )
		self.rosInterface.LocoCommand(	velX = 0.0,
						velY = 0.0,
						omgZ = 0.0,
						commandType = 0,
						ignorable = False )
		time.sleep( 2 )
##		self.rosInterface.LocoCommand( command = 'standToSit', commandType = 1 )
##		

class State1( FSMBrainState ):
	
	def __init__( self, nextState ):
		
		super( State1, self ).__init__( "State1" )
		
		self.nextState = nextState
		
		self.addSubBrain( FindBall( nextState = "TrackingBall" ) )
		self.addSubBrain( TrackingBall( previousState = "FindBall", nextState = "FollowBall" ) )
		self.addSubBrain( FollowBall( kickingState = "None", trackingState = "TrackingBall" ) )
		
		self.setFirstSubBrain( "FindBall" )
		
	def step( self ):
		
		name = self.currSubBrainName
		if name == "None":
			self.SignalChangeSubBrain( self.nextState )

class State2( FSMBrainState ):
	
	def __init__( self, nextState ):
		
		super( State2, self ).__init__( "State2" )
		
		self.nextState = nextState
		
		self.addSubBrain( FindBall( nextState = "TrackingBall2" ) )
		self.addSubBrain( TrackingBall2( previousState = "FindBall", nextState = "FollowBall" ) )
		self.addSubBrain( FollowBall( kickingState = "KickTheBall", trackingState = "TrackingBall2", distanceDecision = 0.3 ) )
		self.addSubBrain( KickTheBall( nextState = "None" ) )
		
		self.setFirstSubBrain( "FindBall" )
		
	def step( self ):
		
		name = self.currSubBrainName
		if name == "None":
			self.SignalChangeSubBrain( self.nextState )			

#findBall = FindBall( nextState = "TrackingBall" )
#trackingBall = TrackingBall( previousState = "FindBall", nextState = "FollowBall" )
#
#followBall = FollowBall( kickingState = "AlignGoal", 
#			 trackingState = "TrackingBall" )

state1 = State1( "AlignGoal" )
align = AlignGoal( nextState = "State2")
state2 = State2( "None" )
#
#kickXSO = KickTheBall( nextState = "TrackingBall" )   

main_brain = MainBrain( "main_brain" )
#main_brain.addSubBrain( findBall )
#main_brain.addSubBrain( trackingBall )
#main_brain.addSubBrain( followBall )
#main_brain.addSubBrain( kickXSO )
#main_brain.addSubBrain( align )
#main_brain.setFirstSubBrain( "FindBall" )

main_brain.addSubBrain( state1 )
main_brain.addSubBrain( align )
main_brain.addSubBrain( state2 )
main_brain.setFirstSubBrain( "State1" )

#main_brain = State2( 'None' )
