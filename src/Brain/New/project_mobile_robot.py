from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

from localization.localMap import createObj

import numpy as np

import time
import math

import rospy

class Stop( FSMBrainState ):

	def __init__( self, nextSubbrain = 'None', time = 5 ):

		super( Stop, self ).__init__( 'Stop' )

		self._time = time
		self._startTime = 0
		self._nextSubbrain = nextSubbrain

	def firstStep( self ):
		rospy.loginfo( "Wait for {} second(s)".format( self._time ) )

		self.rosInterface.local_map( reset = True )

		self.rosInterface.Pantilt(	name=['pan', 'tilt'], position=[0.0, 0.0],
									command = 0
								)

		self._startTime = time.time( )

		self.setGlobalVariable( 'PoleColor', ('magenta', 'yellow') )

	def step( self ):

		if time.time( ) - self._startTime > self._time:
			self.SignalChangeSubBrain( self._nextSubbrain )

class ScanPole( FSMBrainState ):

	def __init__( self, time = 10, nextSubbrain = 'None', kickingState = 'None' ):

		super( ScanPole, self ).__init__( 'ScanPole' )

		self._time = time
		self._startTime = 0.0
		self._nextSubbrain = nextSubbrain
		self._kickingState = kickingState

	def findShootDirection( self, objectDict, color1, color2 ):

		objectDict.setdefault( color1, [] )
		objectDict.setdefault( color2, [] )

		if len( objectDict[ color1 ] ) > 0 and len( objectDict[ color2 ] ) > 0:

			phi1 = objectDict[ color1 ][0].getPolarCoor( )[1]
			phi2 = objectDict[ color2 ][0].getPolarCoor( )[1]
			print math.degrees( phi1 ), math.degrees( phi2 )
			print objectDict[ color1 ][0].getCartCoor(), objectDict[ color2 ][0].getCartCoor()
			return ( phi1 + phi2 ) / 2.0

		# if len( objectDict[color1] ) > 1 and len( objectDict[color2] ) == 0:

		# 	return objectDict[color1][0].getPolarCoor( )[1]

		# if len( objectDict[color1] ) == 0 and len( objectDict[color2] ) > 1:

		# 	return objectDict[color2][0].getPolarCoor( )[1]


	def firstStep( self ):

		self.rosInterface.local_map( reset = True )

		self._startTime = time.time( )

		self.rosInterface.Pantilt( 	pattern = 'findball_pattern',
									command = 1
								)

	def step( self ):

		postDict = self.rosInterface.local_map( reset = False ).postDict

		objectDict = { }

		for name, cart, confidence in zip( postDict.object_name, postDict.pos3D_cart, postDict.object_confidence ):

			objectDict.setdefault(name, []).append( createObj( name, cart.x, cart.y, confidence ) )
		

		poleColor = self.getGlobalVariable( 'PoleColor' )
		color1, color2 = poleColor

		direction = self.findShootDirection( objectDict, color1, color2 )

		if direction is not None:
			rospy.loginfo( 'Found Shoot Direction {}.'.format( math.degrees(direction) ) )
			self.setGlobalVariable( 'curveSlideAngle', direction )
			self.SignalChangeSubBrain( self._nextSubbrain )

		elif time.time( ) - self._startTime > self._time:
			rospy.loginfo( 'Not Found Shoot Direction.' )
			self.SignalChangeSubBrain( self._kickingState )

	def leaveStateCallBack( self ):

		self.rosInterface.Pantilt( command = 3 )

class ScanField( FSMBrainState ):

	def __init__( self, time = 20, nextSubbrain = 'None', failStateSubbrain = 'None' ):

		super( ScanField, self ).__init__( 'ScanField' )

		self._time = time
		self._nextSubbrain = nextSubbrain
		self._failStateSubbrain = failStateSubbrain

	def findEqualtion( self, pointList ):
		print pointList
		A = pointList[0][1] - pointList[1][1]
		B = pointList[1][0] - pointList[0][0]

		C = -1 * ( A*pointList[0][0] + B*pointList[0][1] )

		return A, B, C

	def findSide( self, objectDict ):

		colorOrder = ['blue', 'orange', 'magenta', 'yellow']

		pointList = []

		for name in colorOrder:

			if objectDict.has_key(name) and len(objectDict[name]) > 0:
				pointList.append( objectDict[name][0].getCartCoor( ) )

			if len( pointList ) >= 2:
				break

		if len( pointList ) >= 2:
			A, B, C = self.findEqualtion( pointList )
		else:
			return None, None

		robotSide = C / math.fabs( C )

		if objectDict.has_key('ball') and len(objectDict['ball']) != 0:
			ballPos = objectDict['ball'][0].getCartCoor()
			ballSide = A*ballPos[0] + B*ballPos[1] + C
			ballSide /= math.fabs( ballSide )

		else:
			ballSide = None

		return robotSide, ballSide

	def firstStep( self ):

		self._startTime = time.time( )

		self.rosInterface.Pantilt( 	pattern = 'findball_pattern',
									command = 1
								)

	def step( self ):

		postDict = self.rosInterface.local_map( reset = False ).postDict

		objectDict = { }

		for name, cart, confidence in zip( postDict.object_name, postDict.pos3D_cart, postDict.object_confidence ):

			objectDict.setdefault(name, []).append( createObj( name, cart.x, cart.y, confidence ) )
		
		robotSide, ballSide = self.findSide( objectDict )

		if robotSide is not None and ballSide is not None:
			# self.setGlobalVariable( 'robotSide', robotSide )
			# self.setGlobalVariable( 'ballSide', ballSide )
			if robotSide * ballSide > 0:
				self.SignalChangeSubBrain( self._nextSubbrain )
			else:
				self.SignalChangeSubBrain( self._failStateSubbrain )

		elif time.time( ) - self._startTime > self._time:

			self.setGlobalVariable( 'robotSide', robotSide )
			self.setGlobalVariable( 'ballSide', ballSide )

			self.SignalChangeSubBrain( self._failStateSubbrain )

	def leaveStateCallBack( self ):

		rospy.loginfo( "robot side : {}, ball side : {}".format( self.getGlobalVariable('robotSide'),
																self.getGlobalVariable('ballSide') ) )

		# self.rosInterface.Pantilt( command = 3 )


# main_brain = FSMBrainState( 'main' )
# print "EIEI"
# stop = Stop( nextSubbrain = 'ScanField', time = 2 )
# scan = ScanField( nextSubbrain = 'Stop' )

# main_brain.addSubBrain( stop )
# main_brain.addSubBrain( scan )
# main_brain.setFirstSubBrain( 'Stop' )


