from visionManager.visionModule import KinematicModule, VisionModule
from utility.imageToMessage import cvtImageMessageToCVImage

from newbie_hanuman.msg import HanumanStatusMsg

from scanLine2 import findBoundary, findChangeOfColor, findLinearEqOfFieldBoundary

from std_msgs.msg import Empty
from sensor_msgs.msg import Image

import cv2
import numpy as np

print " ########################################"
print "##                                      ##"
print "#  PLEASE PLACE ROBOT AT 0,2,90 degrees. #"
print "##                                      ##"
print " ########################################"

A = 1.0
B = 0.0
C = -2.0

class Kinematic( KinematicModule ):

	def __init__( self ):
		super( Kinematic, self ).__init__()

		self.objectsMsgType = Image
		self.posDictMsgType = Empty

		self.__imgPoints = []
		self.__panTiltPoses = []
		self.__lineCoef = []

	def kinematicCalculation( self, objMsg, js, rconfig = None ):

		stepSize = 20

		img = cvtImageMessageToCVImage( objMsg )

		gray = img[:,:,0:1]
		gray = np.repeat( gray, 3, axis = 2 )
		
		color_map = img[:, :, 1].copy()

		fieldContour, fieldMask = findBoundary( color_map, 2 )

		points = []
		for p in fieldContour[1:-1:stepSize,0,:]:

			if p[1] == 0:
				continue
			points.append( (p[0], p[1]) )

			cv2.circle( gray, (p[0], p[1]), 4, (0,0,255), -1 )

		cv2.imshow( 'img', gray )
		k = cv2.waitKey( 1 )

		if k == ord( 'w' ) and len( points ) != 0:
			print "capture {} w".format( len(self.__imgPoints) + 1 )
			panTiltPos = np.array( js.position, dtype = np.float64 )
			self.__imgPoints.append( np.vstack(points) )
			self.__panTiltPoses.append( panTiltPos )
			self.__lineCoef.append( [1.0, 0.0, -2.0] )

		elif k == ord( 'a' ) and len( points ) != 0:
			print "capture {} a".format( len(self.__imgPoints) + 1 )
			panTiltPos = np.array( js.position, dtype = np.float64 )
			self.__imgPoints.append( np.vstack(points) )
			self.__panTiltPoses.append( panTiltPos )
			self.__lineCoef.append( [0.0, 1.0, -2.0] )

		elif k == ord( 'd' ) and len( points ) != 0:
			print "capture {} d".format( len(self.__imgPoints) + 1 )
			panTiltPos = np.array( js.position, dtype = np.float64 )
			self.__imgPoints.append( np.vstack(points) )
			self.__panTiltPoses.append( panTiltPos )
			self.__lineCoef.append( [0.0, 1.0, 2.0] )

		elif k == ord( 'q' ):
			print "Delete last instance."
			self.__imgPoints.pop( -1 )
			self.__panTiltPoses.pop( -1 )
			self.__lineCoef.pop( -1 )

		elif k == ord( 's' ) and len( self.__imgPoints ) != 0:
			fn = '/tmp/line.npz'

			print "Save ... to {}".format( fn )

			np.savez( fn,
					imagePoints = self.__imgPoints,
					pantilt = self.__panTiltPoses,
					lineCoef = self.__lineCoef )

		return Empty( )

	def loop( self ):
		pass

vision_module = VisionModule( )
kinematic_module = Kinematic( )





