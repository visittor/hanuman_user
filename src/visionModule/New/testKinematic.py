from visionManager.visionModule import KinematicModule, VisionModule
from newbie_hanuman.msg import HanumanStatusMsg

from std_msgs.msg import Empty

from forwardkinematic import getMatrixForForwardKinematic, setPrefix

import numpy as np
import cv2

setPrefix( 1e+0)

class VisionModule(VisionModule):
	pass

class Kinematic(KinematicModule):

	def __init__(self):
		super(Kinematic, self).__init__()

		intrinMat = np.identity(3)
		intrinMat[0,0] = 640
		intrinMat[1,1] = 480
		intrinMat[0,2] = 320
		intrinMat[1,2] = 240

		self.set_IntrinsicCameraMatrix(intrinMat)

		## Define qc points.
		qcpoint = [ 
					[	0.5, 	0.5, 	0	],
					[	0.5, 	-0.5, 	0	],
					[	-0.5, 	-0.5, 	0	],
					[	-0.5, 	0.5, 	0	] 
				  ]
		self.qcpoint = np.array( qcpoint, dtype = np.float64 )

		self.offsetX = 1.0
		self.offsetY = 0.0

		self.qcpoint[:,0] += self.offsetX
		self.qcpoint[:,1] += self.offsetY

		self.qcpoint_2d = None
		self.image = None

		tranVec = np.zeros( (3,) )
		rotVec = np.zeros( (3,) )
		ground = self.create_transformationMatrix( tranVec, rotVec, 'zyz' )

		self.add_plane( "ground", ground, 
					(-np.inf,np.inf), (-np.inf,np.inf), (-np.inf,np.inf) )

	def kinematicCalculation(self, objMsg, js, rconfig=None):

		npArray = np.fromstring(objMsg.data, dtype=np.uint8).copy()
		self.image = cv2.imdecode(npArray, 1)
		print "..",js.position
		H = getMatrixForForwardKinematic( *js.position )

		self.qcpoint_2d = self.calculate2DCoor( self.qcpoint, "ground",
												HCamera = H )

		return Empty()

	def loop(self):
		if self.qcpoint_2d is None or self.image is None:
			return

		for i in range( len( self.qcpoint_2d ) - 1 ):
			try:
				p1 = tuple( self.qcpoint_2d[ i ].astype( int ) )
				p2 = tuple( self.qcpoint_2d[ i + 1 ].astype( int ) )
				cv2.line( self.image, p1, p2, (255,0,255), 5 )
			except AttributeError as e:
				pass
				
		cv2.imshow( 'image', self.image )
		cv2.waitKey( 1 )

vision_module = VisionModule()
kinematic_module = Kinematic()