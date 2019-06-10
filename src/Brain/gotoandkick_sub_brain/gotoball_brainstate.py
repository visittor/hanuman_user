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

from findball_brainstate import FindBall
from alignball_brainstate import RotateToTheBall, RotateToTheBall2
from followball_brainstate import FollowBall

from slidecurve_brainstate import SlideCurve

from default_config import getParameters

########################################################
#
#	GLOBALS
#

DefaultInitialPreviousDistance = 0.50

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

		super( _IDLE, self ).__init__( '_IDLE' )

	def firstStep( self ):
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )
		self.stopRobotBehavior( )

	def stopRobotBehavior( self ):
		#	stop	
		self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   ignorable =  False )

class GotoBall( FSMBrainState ):

	def __init__( self, nextState = 'None' ):

		super( GotoBall, self ).__init__( 'GotoBall' )

		# self.addSubBrain( _IDLE( ), 'FindBall' )
		self.addSubBrain( FindBall( ), 'FindBall' )

		self.addSubBrain( RotateToTheBall2( failState = 'FindBall',
										successState = "FollowBall",
										lostBallState = 'FindBall' ), 'RotateToTheBall' )
		self.addSubBrain( FollowBall( failState = "RotateToTheBall", 
									successState = 'None',
									lostBallState = 'FindBall' ) )

		self.lookAtBall = False

		self.nextState = nextState

		self.setFirstSubBrain( 'FindBall' ) 

	def initialize( self ):

		#	Get time out from config
		self.scanBallPattern = self.config[ 'PanTiltPlanner' ][ 'ScanBallPattern' ]
		self.confidenceThr = float( getParameters(self.config, 'ChangeStateParameter', 'BallConfidenceThreshold'))

	def firstStep( self ):

		rospy.loginfo( "Enter {} brainstate".format( self.name ) )

		#	Call pattern
		self.rosInterface.Pantilt( command = 1, pattern = self.scanBallPattern )
		
	def step( self ):

		#	Get vision msg
		visionMsg = self.rosInterface.visionManager
		localPosDict = self.rosInterface.local_map( reset = False )
		if localPosDict is None:
			return
		else:
			localPosDict = localPosDict.postDict

		if 'ball' in visionMsg.object_name:
			# idxBallLocalObj = localPosDict.object_name.index( 'ball' )
			# if localPosDict.object_confidence[ idxBallLocalObj ] < self.confidenceThr:
			# 	pass
			
			# self.rosInterface.Pantilt( command = 3 )
			if not self.lookAtBall:
				self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
				rospy.loginfo( "Found ball!!!!" )
				self.lookAtBall = True
			
			if self.currSubBrainName == 'FindBall':
				self.ChangeSubBrain( 'RotateToTheBall' )

		else:
			if self.lookAtBall:
				self.lookAtBall = False
				rospy.loginfo( "LOST ball!!!!" )
				self.rosInterface.Pantilt( command = 1, pattern = self.scanBallPattern )

		if self.currSubBrainName == 'None':
			self.SignalChangeSubBrain( self.nextState )

	def leaveStateCallBack( self ):
		print 'eiei'
		self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
							position=[ 0.0, 0.0 ],
							command=0,
							velocity=[] )

main_brain = GotoBall()