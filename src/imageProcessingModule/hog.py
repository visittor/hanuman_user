#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os

########################################################
#
#	LOCAL IMPORTS
#

import cv2
import numpy as np

########################################################
#
#	GLOBALS
#

DefaultWinSizeTuple = ( 40, 40 )
DefaultBlockSizeTuple = ( 8, 8 )
DefaultBlockStrideTuple = ( 4, 4 )
DefaultCellSizeTuple = ( 4, 4 )
DefaultNumberOfBins = 9

########################################################
#
#	EXCEPTION DEFINITIONS
#

########################################################
#
#	HELPER FUNCTIONS
#

########################################################
#
#	CLASS DEFINITIONS
#

class HogProvider( object ):
    ''' HogProvider class
    '''
    def __init__( self, winSize = DefaultWinSizeTuple, blockSize = DefaultBlockSizeTuple, blockStride = DefaultBlockStrideTuple,
	   		    cellSize = DefaultCellSizeTuple, nBins = DefaultNumberOfBins ):
 
		#	define hog descriptor instance
		self.hogDescriptor = cv2.HOGDescriptor( winSize, blockSize, blockStride, cellSize, nBins )


    def extract( self, imgList ):
        ''' extract function
        '''
        
        #   Initial feature list
        featureList = list()

        #   Loop over image list
        for img in imgList:

            #   Get feature vector
            featureVector = self.hogDescriptor.compute( img )

            #   Transpose to 1-row
            featureVector = featureVector.T

            #   Append to the list
            featureList.append( featureVector )
        
        #   Change to tuple
        featureTuple = tuple( featureList )

        #   Stack row
        featureMatrix = np.vstack( featureTuple )

        #   delete all element in list
        del featureList[:]

        return featureMatrix


            
