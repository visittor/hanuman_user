from visionManager.visionModule import KinematicModule, VisionModule
from utility.imageToMessage import cvtImageMessageToCVImage

from newbie_hanuman.msg import HanumanStatusMsg

from std_msgs.msg import Empty
from sensor_msgs.msg import Image

import cv2
import numpy as np

import binascii

objectsPoint1 = np.zeros( (9*6, 3) )
objectsPoint1[:,:2] = np.mgrid[:6,:9].transpose( 1,2,0 ).reshape(-1,2)[::-1]

OFFSETX1 = 0.385
OFFSETY1 = -0.065
SCALEX1 = 0.021
SCALEY1 = 0.021

objectsPoint1[:,0] *= SCALEX1
objectsPoint1[:,1] *= SCALEY1

objectsPoint1[:,0] += OFFSETX1
objectsPoint1[:,1] += OFFSETY1

objectsPoint2 = np.zeros( (9*6, 3) )
objectsPoint2[:,:2] = np.mgrid[:9,:6].transpose( 2,1,0 ).reshape(-1,2)[::-1]

OFFSETX2 = 0.102
OFFSETY2 = -0.1725
SCALEX2 = 0.021
SCALEY2 = -0.021

objectsPoint2[:,0] *= SCALEX2
objectsPoint2[:,1] *= SCALEY2

objectsPoint2[:,0] += OFFSETX2
objectsPoint2[:,1] += OFFSETY2

objectsPoint3 = np.zeros( (9*6, 3) )
objectsPoint3[:,:2] = np.mgrid[:9,:6].transpose( 2,1,0 ).reshape(-1,2)
objectsPoint3[:,1:2] = objectsPoint3[::-1,1:2]

OFFSETX3 = 0.1015 
OFFSETY3 = 0.158
SCALEX3 = 0.021
SCALEY3 = 0.021

objectsPoint3[:,0] *= SCALEX3
objectsPoint3[:,1] *= SCALEY3

objectsPoint3[:,0] += OFFSETX3
objectsPoint3[:,1] += OFFSETY3

objectsPoint3 = objectsPoint3[::-1]

# print objectsPoint3

# camera_prop = np.load( "/home/visittor/camMat.npz" )
# cameraMatrix = camera_prop[ 'cameraMatrix' ]
# distCoeffs = camera_prop[ 'distCoeffs' ]
# roi = camera_prop[ 'roi' ]

class Kinematic(KinematicModule):

	def __init__(self):
		super(Kinematic, self).__init__()

		self.objectsMsgType = Image
		self.posDictMsgType = Empty

		self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

		self.__objpoints = []
		self.__imgpoints = []
		self.__panTiltPoses = []

	def kinematicCalculation(self, objMsg, js, rconfig=None):
		frame = cvtImageMessageToCVImage( objMsg )

		# frame = cv2.undistort( frame, cameraMatrix, distCoeffs )

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

		frameCopy = frame.copy()
		corners2 = None

		if ret:
			corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
			cv2.drawChessboardCorners(frameCopy, (9,6), corners2, ret)
			# cv2.circle( frameCopy, tuple(corners[0,0]), 3, (0,0,255), -1 )
			# cv2.circle( frameCopy, tuple(corners[1,0]), 3, (0,255,255), -1 )
			# cv2.circle( frameCopy, tuple(corners[8,0]), 3, (255,0,255), -1 )

		cv2.imshow( 'img', frameCopy )
		k = cv2.waitKey( 1 )

		if k == ord( '1' ) and corners2 is not None:
			panTiltPos = np.array( js.position, dtype = np.float64 )
			self.__imgpoints.append( corners2 )
			self.__objpoints.append( objectsPoint1 )
			self.__panTiltPoses.append( panTiltPos )
			print "Capture board 1", len( self.__imgpoints )

		elif k == ord( '2' ) and corners2 is not None:
			panTiltPos = np.array( js.position, dtype = np.float64 )
			self.__imgpoints.append( corners2 )
			self.__objpoints.append( objectsPoint2 )
			self.__panTiltPoses.append( panTiltPos )
			print "Capture board 2", len( self.__imgpoints )

		elif k == ord( '3' ) and corners2 is not None:
			panTiltPos = np.array( js.position, dtype = np.float64 )
			self.__imgpoints.append( corners2 )
			self.__objpoints.append( objectsPoint3 )
			self.__panTiltPoses.append( panTiltPos )
			print "Capture board 3", len( self.__imgpoints )

		elif k == ord( 's' ):
			fn = '/tmp/chessboard.npz'

			print "Save ... to {}".format( fn )

			np.savez( fn,
					imagePoints = self.__imgpoints,
					objectPoints = self.__objpoints,
					pantilt = self.__panTiltPoses )

		return Empty()

	def loop(self):
		pass

vision_module = VisionModule()
kinematic_module = Kinematic()