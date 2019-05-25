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


class FollowBall( FSMBrainState ):

	def __init__( self, previousState = "None", nextState = "None", findBallState = "None" ):
		
		super( FollowBall, self ).__init__( 'FollowBall' )
		
		self.previousState = previousState
		self.nextState = nextState
		
		self.findBallState = findBallState
		
		self.numFrameNotDetectBall = None

		self.velX = None
		self.velY = None
		self.smallTheta = None
		self.distanceToKick = None

	def initialize( self ):
		
		#	Get velocity of x and y
		self.velX = float( self.config[ 'VelocityParameter' ][ 'VelocityXWhenFollowTheBall' ] )
		self.velY = float( self.config[ 'VelocityParameter' ][ 'VelocityYWhenFollowTheBall' ] )
		
		#	Get theta to change state
		self.smallTheta = float( self.config[ 'ChangeStateParameter' ][ 'SmallDegreeToAlignTheBall' ] )

		#	Get neares distance before kick
		self.distanceToKick = float( self.config[ 'ChangeStateParameter' ][ 'NearestDistanceFootballWrtRobot' ] )

		#	Get limit angle
		self.limitTiltAngle = float( self.config[ 'PanTiltPlanner' ][ 'LimitTiltAngleDegree' ] )

	def firstStep( self ):
		
		rospy.loginfo( "Enter {} brainstate".format( self.name ) )	
		
		#	re-initial num frame that not detect ball
		self.numFrameNotDetectBall = 0

		# self.rosInterface.Pantilt( command = 3 )
			
		# self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
		
	def step( self ):
		
		#	Get current vision msg
		visionMsg = self.rosInterface.visionManager
		
		localPosDict = self.rosInterface.local_map( reset = False ).postDict

		#	Get index
		idxBallObj = localPosDict.object_name.index( 'ball' )

		idxBall = visionMsg.object_name.index( 'ball' )

		currentTiltAngle = self.rosInterface.pantiltJS.position[ 1 ]

		# rospy.loginfo( "Tilt angle : {}".format( math.degrees( currentTiltAngle ) ) )
		
		#	Check confidence if model could detect ball
		if localPosDict.object_confidence[ idxBallObj ] > 0.5:
			
			#	get distance and angle
			distanceWrtBall = visionMsg.pos3D_cart[ idxBall ].x
			sideDistanceWrtBall = visionMsg.pos3D_cart[ idxBall ].y

			thetaWrtBall = localPosDict.pos2D_polar[ idxBallObj ].y

			localDistanceX = localPosDict.pos3D_cart[ idxBallObj ].x
			localDistanceY = localPosDict.pos3D_cart[ idxBallObj ].y

			rospy.loginfo( "distance localmap | current distance : {} | {}".format( localDistanceX, distanceWrtBall ) )

			if currentTiltAngle >= math.radians( self.limitTiltAngle ):

				rospy.loginfo( "Final Tilt angle : {}".format( math.degrees( currentTiltAngle ) ) )

				#	Change state to kick immedietly.
				self.stopRobotBehavior()

				self.SignalChangeSubBrain( self.nextState )

			if abs( thetaWrtBall ) > math.radians( self.smallTheta ):
				
				#	Back to align the ball
				self.SignalChangeSubBrain( self.previousState )

			if localDistanceX >= self.distanceToKick:
				
				#	Get side to kick in kicking brain state
				direction = 1 if sideDistanceWrtBall > 0 else -1
				
				self.setGlobalVariable( 'direction', direction )	
			
				self.rosInterface.LocoCommand( velX = self.velX,
							       			   velY = self.velY,
							       			   omgZ = 0.0,
							       			   commandType = 0,
							       			   ignorable = False )
			else:
	
				self.stopRobotBehavior()

				rospy.loginfo( "Finish" )
				rospy.loginfo( "	Angle after stop before kick : {} degrees".format( math.degrees( thetaWrtBall ) ) )
				rospy.loginfo( "	Distance after stop before kick : {} m".format( distanceWrtBall ) )
				rospy.loginfo( "	Select side to kick : {}".format( self.getGlobalVariable( 'direction' ) ) )

				self.SignalChangeSubBrain( self.nextState )
				
			
		else:
			#	it should switch to first state to find the ball
			self.stopRobotBehavior()
			
			self.SignalChangeSubBrain( self.findBallState )
			
			
	def stopRobotBehavior( self ):
		
		#	terminate pantilt
		self.rosInterface.Pantilt( command = 3 )
			
		#	stop
		# self.rosInterface.LocoCommand( velX = 0.0,
		# 			       			   velY = 0.0,
		# 			       			   omgZ = 0.0,
		# 			       			   commandType = 0,
		# 			       			   ignorable = False )		
		self.rosInterface.LocoCommand( command = "StandStill", commandType = 1, 
									   ignorable =  False )