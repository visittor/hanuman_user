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

from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

from newbie_hanuman.msg import postDictMsg


import numpy as np
import math

import time
import rospy

########################################################
#
#	LOCAL IMPORTS
#

########################################################
#
#	GLOBALS
#

Goal_0 = 'goal_0'
Goal_1 = 'goal_1'
InterSectPoint = 'intersect_point'
State_1 = 1
State_2 = 2
State_3 = 3
State_4 = 4

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

class FindGoal( FSMBrainState ):
	
	def __init__( self, nextState ):
		
		super( FindGoal, self ).__init__( 'FindGoal' )
		
		self.nextState = nextState
		
		self.previousTime = None
		
		self.initialPanAngle = None
		
		self.goalPanAngle = None
		
		self.direction = None
		
		self.numFrameIntersect = None
		
		self.goalPolar = None
		
	def firstStep( self ):
		
		rospy.loginfo( "Enter FindGoal Brainstate" )
	
		#	find goal
		self.rosInterface.Pantilt( command = 1, pattern = 'find_goal' )
		
		self.previousTime = time.time()
		
		self.initialPanAngle = 0.0
		self.goalPanAngle = 0.0
		
		self.numFrameIntersect = 0
		
		self.direction = None
		
		self.goalPolar = None
		
	def step( self ):
		
		#	vision msg
		visionMsg = self.rosInterface.visionManager
		
		if Goal_0 in visionMsg.object_name:
		
			self.initialPanAngle = self.rosInterface.pantiltJS.position[ 0 ]
			self.goalPanAngle = self.rosInterface.pantiltJS.position[ 0 ]
			
			self.goalPolar = visionMsg.pos2D_polar[ 1 ].y
			
			while True:
			
				self.rosInterface.Pantilt( name = [ 'pan', 'tilt' ],
						  	   position = [ self.initialPanAngle, math.radians( 5 ) ] )
				
				time.sleep( 0.5 )
				
				#	vision msg after while loop
				visionMsgAfterLoop = self.rosInterface.visionManager
				
				idxIntersectPoint = visionMsgAfterLoop.object_name.index( InterSectPoint )
				
				if Goal_1 in visionMsgAfterLoop.object_name and Goal_0 in visionMsgAfterLoop.object_name:
					
					self.direction = 1
						
					self.rosInterface.Pantilt( name = [ 'pan', 'tilt' ],
						   		   position = [ self.goalPanAngle, math.radians( 5 ) ] )
						   
					time.sleep( 1.0 )
		
					#	terminate current scan
					self.rosInterface.Pantilt( command = 3 )
						
					self.previousTime = time.time()

					break
					
				if visionMsgAfterLoop.object_confidence[ idxIntersectPoint ] > 0.5:
					
					self.numFrameIntersect += 1
					
					if self.numFrameIntersect > 5:
						self.direction = -1
						
						self.rosInterface.Pantilt( name = [ 'pan', 'tilt' ],
									   position = [ self.goalPanAngle, math.radians( 5 ) ] )
						   
						time.sleep( 1.0 )
		
						#	terminate current scan
						self.rosInterface.Pantilt( command = 3 )


						break
						
				self.initialPanAngle -= math.radians( 3 )
				
		if self.direction is not None:
			self.setGlobalVariable( 'direction', self.direction )
			self.setGlobalVariable( 'goalPolar', self.goalPolar )
			self.SignalChangeSubBrain( self.nextState )


class AlignBall( FSMBrainState ):
	
	def __init__( self, previousState ):
		
		super( AlignBall, self ).__init__( "AlignBall" )

		self.previousState = previousState
		
		self.direction = None
		
		self.previousTime = None
		self.previousTimeSlide = None
		
		self.ballPolar = None
		
		self.flagFound = False
		
	def firstStep( self ):
	
		rospy.loginfo( "Enter AlignBall Brainstate" )
		
		#	get direction
		self.direction = self.getGlobalVariable( 'direction' )
		
		self.rosInterface.Pantilt( command = 1, pattern = 'basic_pattern' )
		
		self.previousTime = time.time()
		
		self.previousTimeSlide = time.time()
		
		self.ballPolar = 0
		self.flagFound = False
		
	def step( self ):
		
		visionMsg = self.rosInterface.visionManager
		idxBall = visionMsg.object_name.index( 'ball' )
		
		if visionMsg.object_confidence[ idxBall ] > 0.5 and not self.flagFound:
			
			self.rosInterface.Pantilt( command = 3 )
			
			self.flagFound = True
			
			self.rosInterface.LocoCommand(	velX = -0.1,
							velY = self.direction*0.8,
							omgZ = -1 * self.direction * 0.1,
							commandType = 0 )
							
			self.previousTimeSlide = time.time()
			
		if time.time() - self.previousTime >= 0.5 and self.flagFound:

			self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
			self.previousTime = time.time()
		
			
		if time.time() - self.previousTimeSlide >= 5 and self.flagFound:
			self.rosInterface.LocoCommand(	velX = 0.0,
							velY = 0.0,
							omgZ = 0.0,
							commandType = 0 )
							
			self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
							
							
			self.setGlobalVariable( 'flagSlide', True )
		
			

class AlignGoal( FSMBrainState ):
	
	def __init__( self, nextState ):
		
		super( AlignGoal, self ).__init__( "AlignGoal" )
		
		self.nextState = nextState
		
		self.findGoal = FindGoal( nextState = 'AlignBall' )
		self.alignBall = AlignBall( previousState = 'FindGoal' )
		
		self.addSubBrain( self.findGoal )
		self.addSubBrain( self.alignBall )
		
		self.setFirstSubBrain( "FindGoal" )
		
	def firstStep( self ):

		rospy.loginfo( "Enter AlignGoal Brainstate" )
		
	def step( self ):
	
		flagSlide = self.getGlobalVariable( 'flagSlide' )
		
		if flagSlide is not None:
			if flagSlide:
				self.rosInterface.LocoCommand(	velX = 0.0,
								velY = 0.0,
								omgZ = 0.0,
								commandType = 0 )
				self.SignalChangeSubBrain( self.nextState )
		
#		#	get 
#		ballPolar = self.getGlobalVariable( 'ballPolar' )
#		goalPolar = self.getGlobalVariable( 'goalPolar' )
#		direction = self.getGlobalVariable( 'direction' )
#		
#		if ballPolar is not None and goalPolar is not None and direction is not None:
#			
#			if direction > 0:
#				
#				if ballPolar < goalPolar:
#					self.rosInterface.LocoCommand(	velX = 0.0,
#							velY = 0.0,
#							omgZ = 0.0,
#							commandType = 0 )
#					#self.SignalChangeSubBrain( self.nextState )
#					
#			elif direction < 0:
#				if ballPolar > goalPolar:
#					self.rosInterface.LocoCommand(	velX = 0.0,
#							velY = 0.0,
#							omgZ = 0.0,
#							commandType = 0 )
#					#self.SignalChangeSubBrain( self.nextState )
					
		
		
#class AlignGoal( FSMBrainState ):
#	
#	def __init__( self, nextState, previousState ):
#		
#		super( AlignGoal, self ).__init__( 'AlignGoal' )
#		
#		#	get state
#		self.nextState = nextState
#		self.previousState = previousState
#		
#		#	initial num frame for counter
#		self.numFrames = 0
#		
#		self.numFrameIntersect = 0
#		self.numFrameTwoGoals = 0
#		
#		#	initial state
#		self.state = State_1
#		
#		
#		self.panPosList = [ -10, -20, -30 ]
#		
#		self.direction = 0	#	1 is left, -1 is right
#		
#		self.goalPan = 0
#		
#		self.previousTime = 0
#		
#		self.sign = 0
#		
#		self.goalPostAngle = 0.0
#		self.ballAngle = 0.0
#	
#	def firstStep( self ):
#		
#		#	find goal
#		self.rosInterface.Pantilt( command = 1, pattern = 'find_goal' )
#		
#		#	reinitial numframe
#		self.numFrames = 0
#		
#		self.numFrameIntersect = 0
#		self.numFrameTwoGoals = 0
#		
#		#	reinitial to state 1
#		self.state = State_1
#		self.direction = 0
#		
#		self.goalPan = 0
#		
#		self.previousTime = time.time()
#		
#		self.sign = 0
#		
#	def step( self ):
#		
#		currentTimeStamp = time.time()
#		
#		visionMsg = self.rosInterface.visionManager
#		
#		objNameList = visionMsg.object_name
#		
#		initialPan = 0
#		
#		print "current state : {}".format( self.state )
#		
#		if Goal_0 in objNameList and self.state == State_1:
#			
#			#	terminate current scan
#			self.rosInterface.Pantilt( command = 3 )
#			   
#			time.sleep( 0.5 )
#			
#			initialPan = self.rosInterface.pantiltJS.position[ 0 ]
#			self.goalPan = self.rosInterface.pantiltJS.position[ 0 ]
#						   
#			while True:
#			
#				self.rosInterface.Pantilt( name = [ 'pan', 'tilt' ],
#						  	   position = [ initialPan, math.radians( 5 ) ] )
#				
#				time.sleep( 0.5 )
#				
#				idxIntersectPoint = visionMsg.object_name.index( InterSectPoint )
#				
#				#	vision msg after while loop
#				visionMsgAfterLoop = self.rosInterface.visionManager
#				
#				if Goal_1 in visionMsgAfterLoop.object_name and Goal_0 in visionMsgAfterLoop.object_name:
#					
#					self.direction = 1
#					print "turn left"
#
#					self.state = State_2
#						
#					self.rosInterface.Pantilt( name = [ 'pan', 'tilt' ],
#						   		   position = [ self.goalPan, math.radians( 5 ) ] )
#						   
#					time.sleep( 1.0 )
#		
#					#	terminate current scan
#					self.rosInterface.Pantilt( command = 3 )
#						
#					self.previousTime = time.time()
#
#					break
#					
#				if visionMsgAfterLoop.object_confidence[ idxIntersectPoint ] > 0.5:
#					
#					self.numFrameIntersect += 1
#					
#					if self.numFrameIntersect > 5:
#						self.direction = -1
#						print "turn right"
#
#						self.state = State_2
#						
#						self.rosInterface.Pantilt( name = [ 'pan', 'tilt' ],
#									   position = [ self.goalPan, math.radians( 5 ) ] )
#						   
#						time.sleep( 1.0 )
#		
#						#	terminate current scan
#						self.rosInterface.Pantilt( command = 3 )
#						
#						self.previousTime = time.time()
#
#						break
#						
#				else:
#					
#					self.numFrameIntersect = 0
#				#	increment 3 degree
#				initialPan -= math.radians( 3 )
#		
#		#	state align goal
#		elif self.state == State_2:
#		
##			if currentTimeStamp - self.previousTime >= 0.5:
##					
###				self.rosInterface.Pantilt( command = 2, pattern = Goal_0 )
###				self.previousTime = time.time()
#			
#			idxBall = visionMsg.object_name.index( 'ball' )	
#			
##			
#			if Goal_0 in visionMsg.object_name and visionMsg.object_confidence[ idxBall ] > 0.5:
#				
#				if self.numFrames < 20:
#					self.numFrames += 1
#			else:
#				if self.numFrames > 0:
#					self.numFrames -= 1
#			if self.numFrames > 5:
#				
#				idxGoalPost = visionMsg.object_name.index( Goal_0 )
#				self.goalPostAngle = visionMsg.pos2D_polar[ idxGoalPost ].y
#				ballAngle = visionMsg.pos2D_polar[ idxBall ].y
#				
#				signGoalPost = 1 if self.goalPostAngle > 0 else -1
#				signBall = 1 if ballAngle > 0 else -1
#				
#				if signGoalPost == -1 and signBall == -1:
#					self.state = State_3
##				print self.rosInterface.visionManager.object_name[ idxGoalPost ] 
#				
#				self.rosInterface.LocoCommand(	velX = 0.0,
#								velY = self.direction * 0.5,
#								omgZ = 0.0,
#								commandType = 0 )
#									
#				
##				self.rosInterface.LocoCommand(	velX = 0.0,
##								velY = 0.0,
##								omgZ = -1 * self.direction * 0.2,
##								command = 'OneStepWalk',
##								commandType = 0 )
##
##				self.rosInterface.LocoCommand(	velX = 0.0,
##								velY = 0.0,
##								omgZ = 0.0,
##								commandType = 0 )
##			
##				time.sleep( 3.0 )
#				#self.state = State_3
#				
#				#self.rosInterface.Pantilt( command = 1, pattern = 'basic_pattern_near' )
#				
#				#self.previousTime = time.time()
#				
#		elif self.state == State_3:
#			
#			self.rosInterface.LocoCommand(	velX = 0.0,
#							velY = 0.0,
#							omgZ = 0.0,
#							commandType = 0,
#							ignorable = False )
#								
##		if self.state == State_3:
##			
##			#	ball object index
##			idxBall = self.rosInterface.visionManager.object_name.index( 'ball' )
##			
##			if self.rosInterface.visionManager.object_confidence[ idxBall ] > 0.5:
##			
##				if currentTimeStamp - self.previousTime >= 0.5:
##					
##					self.rosInterface.Pantilt( command = 2, pattern = 'ball' )
##					self.previousTime = time.time()
##					
##					
##				thetaWrtRobotDegree = self.rosInterface.visionManager.pos2D_polar[ idxBall ].y
##				
##				print math.degrees( thetaWrtRobotDegree )
##				
##				sign = 1 if thetaWrtRobotDegree > 0 else -1
##				
##				if thetaWrtRobotDegree > math.radians( 10 ):
##					self.rosInterface.LocoCommand(	velX = 0.0,
##									velY = -0.1,
##									omgZ = sign * 0.2,
##									commandType = 0,
##									ignorable = False )
##				else:
##					self.rosInterface.LocoCommand(	velX = 0.0,
##									velY = 0.0,
##									omgZ = 0.0,
##									commandType = 0,
##									ignorable = False )
##
##				
###
##					#	get side
##					xMagnitude = self.rosInterface.visionManager.pos3D_cart[ idxBall ].x
##					yMagnitude = self.rosInterface.visionManager.pos3D_cart[ idxBall ].y
##
##					#	TODO might check distance x if it close robot will kick na ja
##
##					#	one step
##					self.rosInterface.LocoCommand(	velX = 1.0,
##									velY = 0.0,
##									omgZ = 0.0,
##									command = 'OneStepWalk' )
##					time.sleep( 5.0 )
##
##					if self.direction > 0:
##						self.rosInterface.LocoCommand( command = "LeftKick", commandType = 1 )
##					else:
##						self.rosInterface.LocoCommand( command = "RightKick", commandType = 1 )
##
##					time.sleep( 2 )
##
##					print "Finish"
#
##			
##main_brain = AlignGoal( "Test", "Naja" )
##						   
#			
#			
