#!/usr/bin/env python


import numpy as np
import cv2

import pickle

def loadPickle( filePathStr ):
	
	with open( filePathStr, 'r' ) as f:
		
		obj = pickle.load( f )
		
	return obj
		
winSize = ( 40, 40 )
blockSize = ( 8, 8 )
blockStride = ( 4, 4 )
cellSize = ( 4, 4 )
nBins = 9

imgPathStr = '/home/neverholiday/work/ball_detector/raw_data/fibo_field_ball_1/frame0013.jpg'
modelPathStr = '/home/neverholiday/work/ball_detector/model/model_gray.pkl'

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


			cv2.imshow( 'img', visualizeImage )
			k = cv2.waitKey( 24 )
			
			
cv2.destroyAllWindows()
