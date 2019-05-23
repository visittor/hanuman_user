from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

from localization.localMap import createObj

import numpy as np

import time
import math

import rospy

class Stop( FSMBrainState ):

	def __init__( self, nextSubrain = 'None', time = 5 ):

		super( Stop, self ).__init__( 'Stop' )

		self._time = time
		self._startTime = 0
		self._nextSubbrain = nextSubrain

	def firstStep( self ):
		rospy.loginfo( "Wait for {} second(s)".format( self._time ) )

		self.rosInterface.Pantilt(	name=['pan', 'tilt'], position=[0.0, 0.0],
									command = 0
								)

		self._startTime = time.time( )

	def step( self ):

		if time.time( ) - self._startTime > self._time:
			self.SignalChangeSubBrain( self._nextSubbrain )


class ScanGoal( FSMBrainState ):

	def __init__( self, time = 10, nextSubrain = 'None' ):

		super( ScanGoal, self ).__init__( 'ScanGoal' )

		self._time = time
		self._startTime = 0
		self._nextSubbrain = nextSubrain

		self.setGlobalVariable( 'curveSlideAngle', 0.0 )

	def findShootDirection( self, objectDict ):

		if len( objectDict[ 'goal' ] ) > 1:
			nearestGoal = min( objectDict[ 'goal' ], key = lambda obj: obj.getPolarCoor()[0] )

			_, phi1 = nearestGoal.getPolarCoor( )

			minDist = 6

			for obj in objectDict[ 'goal' ]:
				dist = nearestGoal.distance( obj )
				if dist < minDist and dist != 0:
					minDist = dist
					_, phi2 = obj.getPolarCoor( )
					# print "CASE 1", math.degrees( phi1 ), math.degrees( phi2 )
					return ( phi1 + phi2 ) / 2.0

	def firstStep( self ):

		self.rosInterface.local_map( reset = True )

		self._startTime = time.time( )

		self.rosInterface.Pantilt(	pattern = 'scan_horizon',
									command = 1
								)

	def step( self ):

		postDict = self.rosInterface.local_map( reset = False ).postDict

		objectDict = { 'field_corner' : [], 'goal' : [], 'ball' : [] }

		for name, cart, confidence in zip( postDict.object_name, postDict.pos3D_cart, postDict.object_confidence ):

			objectDict[ name ].append( createObj( name, cart.x, cart.y, confidence ) )

		direction = self.findShootDirection( objectDict )

		if direction is not None:
			rospy.loginfo( 'Found Shoot Direction {}.'.format( math.degrees(direction) ) )
			self.setGlobalVariable( 'curveSlideAngle', direction )
			self.SignalChangeSubBrain( self._nextSubbrain )

		elif time.time( ) - self._startTime > self._time:
			rospy.loginfo( 'Not Found Shoot Direction.' )
			self.SignalChangeSubBrain( self._nextSubbrain )

	def leaveStateCallBack( self ):

		self.rosInterface.Pantilt( command = 3 )


main_brain = FSMBrainState( 'main' )
print "EIEI"
stop = Stop( nextSubrain = 'ScanGoal' )
scan = ScanGoal( nextSubrain = 'Stop' )

main_brain.addSubBrain( stop )
main_brain.addSubBrain( scan )
main_brain.setFirstSubBrain( 'ScanGoal' )