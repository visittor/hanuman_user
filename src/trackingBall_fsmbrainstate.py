#!/usr/bin/env python

from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

from newbie_hanuman.msg import postDictMsg

import numpy as np

import time
import rospy
 
class TrackingBall( FSMBrainState ):
	
	#	state name
	FIND_BALL = 0
	LOOK_AT_BALL = 1

	def __init__( self, nextState = None ):

		super( TrackingBall, self ).__init__( 'TrackingBall' )
		self.nextState = nextState

		self.currentState = None

		self.loopTimeLookAtBall = 0.5
		
	def firstStep( self ):
		'''
			first step before execute this brain
		'''

		rospy.loginfo( "Enter tracking ball state" )

		#	set first state : find ball
		self.currentState = self.FIND_BALL

		self.previousCommandTime = None
		self.rosInterface.Pantilt(	pattern="basic_pattern",
									command=1 )

	def step( self ):
		
		currentStepTime = time.time()

		#if timeStamp - self.initTime >= 3:
		if self.rosInterface.visionManager.ball_confidence == True and self.currentState == self.FIND_BALL:
			
			rospy.loginfo( "Detect the ball!!!!" )

			self.rosInterface.Pantilt( command = 3 )
			
			self.currentState = self.LOOK_AT_BALL

			self.previousCommandTime = currentStepTime

		if self.currentState == self.LOOK_AT_BALL and currentStepTime - self.previousCommandTime >= self.loopTimeLookAtBall:

			self.rosInterface.Pantilt( command = 2 )
			self.previousCommandTime = currentStepTime

			if self.rosInterface.visionManager.ball_confidence == False:
				
				self.rosInterface.Pantilt(	pattern="basic_pattern",
											command=1 )
				
				self.currentState = self.FIND_BALL


class FollowBall( FSMBrainState ):

	def __init__( self, nextState = None ):
		super( FollowBall, self ).__init__( 'FollowBall' )

		#	Set next state
		self.nextState = nextState

	def firstStep( self ):

		rospy.loginfo( "Enter follow ball state" )

		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.nextState )

	def step( self ):
		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.nextState )
		

class MainBrain( FSMBrainState ):
	# def firstStep(self):
	# 	self.rosInterface.Pantilt(	pattern="basic_pattern",
	# 								command=1)

	def end(self):

		self.rosInterface.Pantilt( command = 3 )
		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = 0.0,
										commandType = 0,
										ignorable = False )
		time.sleep(1)


trackingBall = TrackingBall( nextState = "FollowBall" )
followBall = FollowBall( nextState = "TrackingBall" )   

main_brain = MainBrain( "main_brain" )
main_brain.addSubBrain( trackingBall )
main_brain.addSubBrain( followBall )
main_brain.setFirstSubBrain( "TrackingBall" )