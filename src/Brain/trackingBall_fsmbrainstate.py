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

		#	initial num frame for check it's the ball
		self.numFrame = 0
		
	def firstStep( self ):
		'''
			first step before execute this brain
		'''

		rospy.loginfo( "Enter tracking ball state" )

		#	set first state : find ball
		self.currentState = self.FIND_BALL

		#	re-initial num frame for check it's the ball
		self.numFrame = 0
		
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

			self.SignalChangeSubBrain( self.nextState )

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

		#	set time to look at the ball
		self.loopLookAtBall = 0.5
		self.previousTime = 0.0

	def firstStep( self ):

		rospy.loginfo( "Enter follow ball state" )

		#	re-initialize time to look at the ball
		self.previousTime = 0.0

		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.nextState )

	def step( self ):

		#	initial current step time
		currentStepTime = time.time()

		if self.rosInterface.visionManager.ball_confidence == False:
			self.SignalChangeSubBrain( self.nextState )
		
		#	look at the ball
		if currentStepTime - self.previousTime >= self.loopLookAtBall:

			#	command of pantilt to tracking the ball
			self.rosInterface.Pantilt( command = 2 )

			#	save time
			self.previousTime = currentStepTime

		#	get theta error from vision manger
		ballPolarArray = self.rosInterface.visionManager.ball_polar
		thetaBallRefToRobot = ballPolarArray[ 1 ]

		rospy.logdebug( "theta of the ball wrt. robot is {}".format( thetaBallRefToRobot ) )

		#	check orientation of robot and the ball
		#	if angle of the ball wrt. robot more than 5, activate lococommand to turn it
		if abs( thetaBallRefToRobot ) >= np.deg2rad( 5 ):
			
			rospy.loginfo( "Robot is turn around" )

			#	get direction for rotate
			sign = thetaBallRefToRobot / abs( thetaBallRefToRobot )

			#	activate lococommand
			self.rosInterface.LocoCommand(	velX = 0.0,
											velY = 0.0,
											omgZ = sign * 0.1,
											commandType = 0,
											ignorable = False )
		
		else:
			
			rospy.loginfo( "Robot's position have aligned wrt the ball!!!!" )
			rospy.loginfo( "Robot is going straight to the ball!!!" )

			#	stop when robot is align with the ball
			self.rosInterface.LocoCommand(	velX = 0.3,
											velY = 0.0,
											omgZ = 0.0,
											commandType = 0,
											ignorable = False )

class TurnRighToBall( FSMBrainState ):

	def __init__( self ):
		
		#	Set the name
		super( TurnRighToBall, self ).__init__( "TurnRighToBall" )

	def firstStep( self ):

		self.rosInterface.LocoCommand(	velX = 0.0,
										velY = 0.0,
										omgZ = -0.1,
										commandType = 0,
										ignorable = False )

	def step( self ):
		pass

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
		pass

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
		pass
   

	 	

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