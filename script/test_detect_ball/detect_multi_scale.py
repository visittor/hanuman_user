#!/usr/bin/env python


import numpy as np
import cv2

import pickle

import optparse

def loadPickle( filePathStr ):
	
	with open( filePathStr, 'r' ) as f:
		
		obj = pickle.load( f )
		
	return obj

def main():


	parser = optparse.OptionParser( usage='command [imgPathStr] [modelPathStr]' )

	( options, args ) = parser.parse_args()

	if len( args ) > 2:
		parser.error( "require 2 arguments" )
	elif len( args ) < 2:
		parser.error( "need more argumnent, require 2 arguments" )


	winSize = ( 40, 40 )
	blockSize = ( 8, 8 )
	blockStride = ( 4, 4 )
	cellSize = ( 4, 4 )
	nBins = 9

	# imgPathStr = '/home/neverholiday/work/ball_detector/raw_data/fibo_field_ball_1/frame0013.jpg'
	# modelPathStr = '/home/neverholiday/work/ball_detector/model/model_gray.pkl'

	imgPathStr = args[ 0 ]
	modelPathStr = args[ 1 ]

	scaleTime = 3

	boundingWidth = 40
	boundingHeight = 40

	strideX = 16
	strideY = 16

	imgList = list()
	coordinate = list()

	cv2.namedWindow( 'img', cv2.WINDOW_NORMAL )

	imgColor = cv2.imread( imgPathStr )
	img = cv2.imread( imgPathStr, 0 )

	imgWidth = img.shape[ 1 ]
	imgHeight = img.shape[ 0 ]

	modelSVM = loadPickle( modelPathStr )

	for scale in xrange( scaleTime ):
		
		if scale > 0:
			imgColor = cv2.pyrDown( imgColor )
			img = cv2.pyrDown( img )
			
		imgList.append( imgColor )
		
		print "current shape : {}".format( imgColor.shape )
		imgWidth = imgColor.shape[ 1 ]
		imgHeight = imgColor.shape[ 0 ]
			
			
		for j in xrange( 0, imgHeight - boundingHeight, strideY ):
			for i in xrange( 0, imgWidth - boundingWidth, strideX ):
				visualizeImage = imgColor.copy()

				hogDescriptor = cv2.HOGDescriptor( winSize, blockSize, blockStride, cellSize, nBins )

				cv2.rectangle( visualizeImage, ( i, j ), ( i+boundingWidth, j+boundingHeight ), ( 255, 0, 0 ), 1 )
				
				#	get roi
				imgGrayROI = img[ j : j + boundingHeight, i : i + boundingWidth ].copy()

				# #	resize to 40, 40
				# imgGrayROIResized = cv2.resize( imgGrayROI, ( 40, 40 ) )

				#	compute hog 
				hogVector = hogDescriptor.compute( imgGrayROI ).T

				predictVal = modelSVM.predict_proba( hogVector )[ 0, 1 ]
				print predictVal

				if predictVal > 0.7:
					coordinate.append( ( ( i, j ), ( i + boundingWidth, j + boundingHeight ) ) )

				for coor in coordinate:
					cv2.rectangle( visualizeImage, coor[ 0 ], coor[ 1 ], ( 0, 0, 255 ), 2 )

				cv2.imshow( 'img', visualizeImage )
				cv2.imshow( 'gray', imgGrayROI )
				k = cv2.waitKey( 1 )
		cv2.waitKey( 0 )
		
		#	delete element in list
		del coordinate[ : ]
				
				
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()