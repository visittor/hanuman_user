from visionManager.visionModule import KinematicModule, VisionModule
from utility.imageToMessage import cvtImageMessageToCVImage

from newbie_hanuman.msg import HanumanStatusMsg

from std_msgs.msg import Empty
from sensor_msgs.msg import Image

import cv2
import numpy as np

import binascii

def createPanTiltChunk( pan, tilt ):
	dataArray = np.array( [pan, tilt], dtype = np.dtype("<f8") )
	dataStr = dataArray.tostring()
	length = "".join( [ chr(0), chr(0), chr(0), chr(16) ] )
	chunkType = 'paNt'

	crc = binascii.crc32( length + chunkType + dataStr )
	crcStr = "".join([ chr((crc & (255 << i*8))>>i*8) for i in range( 3, -1, -1 ) ])

	return length + chunkType + dataStr + crcStr

def embedToPNGBuffer( dataChunk, pngBuffer ):
	firstDataLength = sum([ord(n)<<8*(3 - 1) for i,n in enumerate( pngBuffer[8:12] )])
	splitIndx = 8 + firstDataLength + 12

	return pngBuffer[:splitIndx] + dataChunk + pngBuffer[splitIndx:]

def getEmbededData( buffer ):
	chunkIdx = buffer.find( "paNt" ) - 4
	chunkData = buffer[ chunkIdx : chunkIdx + 28 ]
	return np.frombuffer( chunkData[8:24], dtype = np.dtype("<f8") )

class Kinematic(KinematicModule):

	def __init__(self):
		super(Kinematic, self).__init__()

		self.objectsMsgType = Image
		self.posDictMsgType = Empty

		self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

		self.__count = 1

	def kinematicCalculation(self, objMsg, js, rconfig=None):
		frame = cvtImageMessageToCVImage( objMsg )
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		
		ret, corners = cv2.findChessboardCorners(gray, (8,6), None)

		frameCopy = frame.copy()
		if ret:
			corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), self.criteria)
			cv2.drawChessboardCorners(frameCopy, (8,6), corners2, ret)

		cv2.imshow( 'img', frameCopy )
		k = cv2.waitKey( 1 )

		if k == ord( 's' ):
			buf = cv2.imencode( '.png', frame )[1].tostring()
			embedData = createPanTiltChunk( js.position[0], js.position[1] )
			embededBufferStr = embedToPNGBuffer( embedData, buf )
			embededBufferArr = np.frombuffer( embededBufferStr, dtype=np.uint8 )
			embededBufferArr.tofile( "~/chessboard/%d.png" % self.__count )
			self.__count += 1

		return Empty()

	def loop(self):
		pass

vision_module = VisionModule()
kinematic_module = Kinematic()