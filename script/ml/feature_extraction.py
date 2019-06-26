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

########################################################
#
#	EXCEPTION DEFINITIONS
#

########################################################
#
#	HELPER FUNCTIONS
#

def loadImageList( pathImageDir ):

    print "Load image from : {}".format( pathImageDir )

    #	get abs path
    absFramePathStr = os.path.abspath( pathImageDir  )

    #	get list of image directory
    absImagePathList = os.listdir( absFramePathStr )

    #	sort first
    absImagePathList.sort()

    #	bring abs path to this directory and add name of each image
    absImagePathList = map( lambda imageName : os.path.join( absFramePathStr, imageName ), absImagePathList )

    return absImagePathList 

########################################################
#
#	CLASS DEFINITIONS
#

class FeatureExtractor( object ):

    def __init__( self, pathImageDir ):
        
        self.pathImageDir = pathImageDir
        self.imageList = loadImageList( pathImageDir )

        self.extractor = self.setExtractor()

    def extract( self, img ):
        ''' Override here
        '''
        return img.reshape( -1 )

    def setExtractor( self ):
        ''' Override here
        '''
        return None

    def doExtract( self, isGray = True ):
        
        featureVectorList = list()

        for imageFileName in self.imageList:

            if isGray:
                img = cv2.imread( imageFileName, 0 )

            else:
                img = cv2.imread( imageFileName )
            
            featureVector = self.extract( img )
            featureVectorList.append( featureVector )

        featureArray = np.vstack( tuple( featureVectorList ) )

        return featureArray


class HOG( FeatureExtractor ):

    def __init__( self, pathImageDir, winSize = ( 40, 40 ), blockSize = ( 8, 8 ), blockStride = ( 4, 4 ), cellSize = ( 4, 4 ), nBins = 9 ):

        self.winSize = winSize
        self.blockSize = blockSize
        self.blockStride = blockStride
        self.cellSize = cellSize
        self.nBins = nBins

        super( HOG, self ).__init__( pathImageDir )

    def setExtractor( self ):

        hogDescriptor = cv2.HOGDescriptor( self.winSize, self.blockSize, self.blockStride, self.cellSize, self.nBins )
        return hogDescriptor

    def extract( self, img ):

        featureVector = self.extractor.compute( img ).T
        return featureVector

class ColorHistrogram( HOG ):

    def __init__( self, pathImageDir, hitSize = 32 , winSize = ( 40, 40 ), blockSize = ( 8, 8 ), blockStride = ( 4, 4 ), cellSize = ( 4, 4 ), nBins = 9 ):
        
        self.hitSize = hitSize
        super( ColorHistrogram, self ).__init__( pathImageDir, winSize, blockSize, blockStride, cellSize, nBins )

    def getHistogram( self, img, normalize = True ):

        hist = cv2.calcHist( [img], [0], None, [self.hitSize], [0,256] ).reshape( 1, -1 )
        if normalize:
            hist = hist / np.max( hist )
        return hist

    def extract( self, img ):

        hogFeatureVector = super( ColorHistrogram, self ).extract( img )

        # print hogFeatureVector.shape

        #   Calculate color histogram
        hist = self.getHistogram( img )
        # print hist.shape

        combineFeature = np.hstack( ( hogFeatureVector, hist ) )
        
        return combineFeature
