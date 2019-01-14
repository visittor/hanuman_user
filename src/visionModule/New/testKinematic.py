from visionManager.visionModule import KinematicModule, VisionModule

from std_msgs.msg import Empty

from forwardkinematic import *

from utility.imageToMessage import cvtImageMessageToCVImage

import numpy as np
import cv2

setPrefix( 1e+0)

ORIGINAL_DIMENSION = [ 0.46, 0.05, 0.02, 0.03, 0.1, 0.02, 0.005, 5.0 ]
CALIBRATED_DIMENSION = []

class VisionModule(VisionModule):
	pass

class Kinematic(KinematicModule):

	def __init__(self):
		super(Kinematic, self).__init__()

		# intrinMat = np.identity(3)
		# intrinMat[0,0] = 640
		# intrinMat[1,1] = 480
		# intrinMat[0,2] = 320
		# intrinMat[1,2] = 240
		camera_prop = np.load( "/home/visittor/camMat.npz" )
		self.cameraMatrix = camera_prop[ 'cameraMatrix' ]
		self.distCoeffs = camera_prop[ 'distCoeffs' ]
		roi = camera_prop[ 'roi' ]


		self.set_IntrinsicCameraMatrix(self.cameraMatrix)

		# ## Define qc points.
		# qcpoint = [ 
		# 			[	0.25, 	0.25, 	0	],
		# 			[	0.25, 	-0.25, 	0	],
		# 			[	-0.25, 	-0.25, 	0	],
		# 			[	-0.25, 	0.25, 	0	] 
		# 		  ]
		# self.qcpoint = np.array( qcpoint, dtype = np.float64 )

		# self.offsetX = 0.5
		# self.offsetY = 0.0

		# self.qcpoint[:,0] += self.offsetX
		# self.qcpoint[:,1] += self.offsetY
		objectsPoint1 = np.zeros( (8*6, 3) )
		objectsPoint1[:,:2] = np.mgrid[:6,:8].transpose( 1,2,0 ).reshape(-1,2)[::-1]

		OFFSETX1 = 0.3
		OFFSETY1 = -0.025 * 3.5
		SCALEX1 = 0.0245
		SCALEY1 = 0.0245

		objectsPoint1[:,0] *= SCALEX1
		objectsPoint1[:,1] *= SCALEY1

		objectsPoint1[:,0] += OFFSETX1
		objectsPoint1[:,1] += OFFSETY1

		objectsPoint2 = np.zeros( (8*6, 3) )
		objectsPoint2[:,:2] = np.mgrid[:8,:6].transpose( 2,1,0 ).reshape(-1,2)[::-1]

		OFFSETX2 = 0.085
		OFFSETY2 = -0.16
		SCALEX2 = 0.0245
		SCALEY2 = -0.0245

		objectsPoint2[:,0] *= SCALEX2
		objectsPoint2[:,1] *= SCALEY2

		objectsPoint2[:,0] += OFFSETX2
		objectsPoint2[:,1] += OFFSETY2

		objectsPoint3 = np.zeros( (8*6, 3) )
		objectsPoint3[:,:2] = np.mgrid[:8,:6].transpose( 2,1,0 ).reshape(-1,2)
		objectsPoint3[:,1:2] = objectsPoint3[::-1,1:2]

		OFFSETX3 = 0.0585
		OFFSETY3 = 0.16
		SCALEX3 = 0.0245
		SCALEY3 = 0.0245

		objectsPoint3[:,0] *= SCALEX3
		objectsPoint3[:,1] *= SCALEY3

		objectsPoint3[:,0] += OFFSETX3
		objectsPoint3[:,1] += OFFSETY3

		self.qcpoint = np.vstack( (objectsPoint1, objectsPoint2, objectsPoint3) )

		self.qcpoint_2d_1 = None
		self.qcpoint_2d_2 = None
		self.image = None

		tranVec = np.zeros( (3,) )
		rotVec = np.zeros( (3,) )
		ground = self.create_transformationMatrix( tranVec, rotVec, 'zyz' )

		self.add_plane( "ground", ground, 
					(-np.inf,np.inf), (-np.inf,np.inf), (-np.inf,np.inf) )

	def kinematicCalculation(self, objMsg, js, rconfig=None):

		npArray = np.fromstring(objMsg.data, dtype=np.uint8).copy()
		self.image = cvtImageMessageToCVImage( objMsg )
		# self.image = cv2.undistort( self.image, self.cameraMatrix, self.distCoeffs )
		# print "..",js.position
		resetConfiguration()

		H = getMatrixForForwardKinematic( *js.position )

		self.qcpoint_2d_1 = self.calculate2DCoor( self.qcpoint, "ground",
												HCamera = H )

		setNewRobotConfiguration( *ORIGINAL_DIMENSION )

		H = getMatrixForForwardKinematic( *js.position )

		self.qcpoint_2d_2 = self.calculate2DCoor( self.qcpoint, "ground",
												HCamera = H )
		# print self.qcpoint_2d
		return Empty()

	def loop(self):
		# print "..", self.qcpoint_2d, self.image is None
		if self.qcpoint_2d_1 is None or self.image is None:
			return

		# for i in range( 4 ):
		# 	try:
		# 		p1 = tuple( self.qcpoint_2d[ i % 4 ].astype( int ) )
		# 		p2 = tuple( self.qcpoint_2d[ (i + 1)%4 ].astype( int ) )
		# 		cv2.line( self.image, p1, p2, (255,0,255), 5 )
		# 	except AttributeError as e:
		# 		pass
		for p in self.qcpoint_2d_1:
			cv2.circle( self.image, tuple( p.astype(int) ), 2, (0,0,255), -1 )

		for p in self.qcpoint_2d_2:
			cv2.circle( self.image, tuple( p.astype(int) ), 2, (0,255,0), -1 )
				
		cv2.imshow( 'image', self.image )
		cv2.waitKey( 1 )

vision_module = VisionModule()
kinematic_module = Kinematic()