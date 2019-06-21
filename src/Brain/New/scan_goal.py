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

		self.rosInterface.Pantilt(	name=['pan', 'tilt'], position=[0.0, 0.0],
									command = 0
								)

		self._startTime = time.time( )

	def step( self ):

		if time.time( ) - self._startTime > self._time:
			self.SignalChangeSubBrain( self._nextSubbrain )


class ScanGoal( FSMBrainState ):

	def __init__( self, time = 10, nextSubbrain = 'None', kickingState = "None" ):

		super( ScanGoal, self ).__init__( 'ScanGoal' )

		self._time = time
		self._startTime = 0
		self._nextSubbrain = nextSubbrain

		self.kickingState = kickingState

		self.setGlobalVariable( 'curveSlideAngle', 0.0 )

	def postDict2ObjDict( self, postDict ):

		objectDict = { 'field_corner' : [], 'goal' : [], 'ball' : [] }

		for name, cart, confidence in zip( postDict.object_name, postDict.pos3D_cart, postDict.object_confidence ):

			objectDict.setdefault( name, [] ).append( createObj( name, cart.x, cart.y, confidence ) )

		return objectDict

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

		if len( objectDict['goal'] ) == 1:
			if len( objectDict['field_corner'] ) > 0:
				goal = objectDict['goal'][0]

				minDist = 6

				for corner in objectDict['field_corner']:
					dist = goal.distance(corner)

					if dist < minDist:
						_, phi1 = goal.getPolarCoor( )
						_, phi2 = corner.getPolarCoor( )
						return phi1 - ((( phi1 + phi2 ) / 2) - phi1)

			return objectDict['goal'][0].getPolarCoor( )[1]

	def firstStep( self ):

		postDict = self.rosInterface.local_map( reset = False ).postDict

		objectDict = self.postDict2ObjDict( postDict )

		if len(objectDict[ "goal" ]) > 0:
			_, phi = objectDictp["goal"][0].getPolarCoor( )

		elif len(objectDict[ "field_corner" ]) > 0:
			_, phi = objectDict["field_corner"][0].getPolarCoor( )


		panAng = math.radians( 30 ) if phi > 0 else math.radians( -30 )
		self.rosInterface.Pantilt(	name=[ 'pan', 'tilt' ],
									position=[ panAng, math.radians(20.0) ],
									command=0,
									velocity=[100, 100] )

		time.sleep( 1.0 )

		self.rosInterface.local_map( reset = True )

		self._startTime = time.time( )

		self.rosInterface.Pantilt(	pattern = 'scan_horizon',
									command = 1
								)

	def step( self ):

		postDict = self.rosInterface.local_map( reset = False ).postDict

		objectDict = self.postDict2ObjDict( postDict )

		direction = self.findShootDirection( objectDict )

		if direction is not None:
			rospy.loginfo( 'Found Shoot Direction {}.'.format( math.degrees(direction) ) )
			self.setGlobalVariable( 'curveSlideAngle', direction )
			self.SignalChangeSubBrain( self._nextSubbrain )

		elif time.time( ) - self._startTime > self._time:
			rospy.loginfo( 'Not Found Shoot Direction.' )
			self.SignalChangeSubBrain( self.kickingState )

	def leaveStateCallBack( self ):

		self.rosInterface.Pantilt( command = 3 )


# main_brain = FSMBrainState( 'main' )
# print "EIEI"
# stop = Stop( nextSubbrain = 'ScanGoal' )
# scan = ScanGoal( nextSubbrain = 'Stop' )

# main_brain.addSubBrain( stop )
# main_brain.addSubBrain( scan )
# main_brain.setFirstSubBrain( 'ScanGoal' )