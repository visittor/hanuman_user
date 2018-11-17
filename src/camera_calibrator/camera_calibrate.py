#!/usr/bin/env python

import cv2
import numpy as np

import time

class CameraCalibration( object ):
    """
    Camera calibration class provide tool for finding intrinsic camera matrix
    argument :
        patternSize : (tuple) size of chessboard pattern, this tuple contain integer size
    """

    def __init__( self, patternSize ):
        
        #   Defind criteria 
        #   TODO : I'm not sure what is it. At this moment I do not understand
        self.criteria = ( cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1 )

        #   TODO : Program object point later, I copied from openCV document
        #   Create empty list of 2D point (pixel-space) and 3D point (real_world-space)
        # # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        # objp = np.zeros((6*7,3), np.float32)
        # objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
        # # Arrays to store object points and image points from all the images.
        # objpoints = [] # 3d point in real world space
        # imgpoints = [] # 2d points in image plane.

        #   defind corner position
        self.corners = None
        self.cornersSubPixel = None
        self.patternSize = patternSize

    def findChessboardCorners( self, img ):
        """
        Find corner of the chess board with function cv2.findChessboardCorners
        argument :
            img : (numpy.array) Gray image from camera video of image
        return :
            status : (bool) Return True if library can find chess board corner
       """

        #   Find corner and get status
        status, self.corners = cv2.findChessboardCorners( img, self.patternSize, cv2.CALIB_CB_ADAPTIVE_THRESH |
                                                          cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK )
        return status

    def findSubpixelChessBoardCorner( self, img ):
        """
        argument :
            img : (numpy.array) Gray image from camera video of image
        Find sub-pixel from chess board corner
        """
        self.cornersSubPixel = cv2.cornerSubPix( img, self.corners, ( 11, 11 ), ( -1, -1 ), self.criteria )

if __name__ == '__main__':

    cap = cv2.VideoCapture( 1 )

    calibrator = CameraCalibration( ( 7, 6 ) )
        
    while True:
        
        _, frame = cap.read()

        grayImage = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )

        ret = calibrator.findChessboardCorners( grayImage )
        
        if ret == True:
            
            calibrator.findSubpixelChessBoardCorner( grayImage )

            #   Draw !
            cv2.drawChessboardCorners( grayImage, calibrator.patternSize, calibrator.cornersSubPixel, ret )

        cv2.imshow( "frame", grayImage )
        k = cv2.waitKey( 1 )

        if k == ord( 'q' ):
            break
        
    cap.release()
    cv2.destroyAllWindows()