import cv2
import numpy as np 
import time
import sys

def findFirstValue2D( array, val, axis = 0, invalid_val = -1):
    mask = array == val
    return np.where( mask.any( axis = axis ), mask.argmax( axis = axis), invalid_val )


def findBoundary( colorMap, colorID, flip = False ):
    colorMap = colorMap if not flip else colorMap[::-1]

    y = findFirstValue2D( colorMap, colorID, axis = 0 )
    x = np.arange( colorMap.shape[1] )

    y[0] = colorMap.shape[0] - 1
    y[-1] = colorMap.shape[0] - 1
    x[0] = 0
    x[-1] = colorMap.shape[1] - 1

    contour = np.vstack( ( x, y ) ).transpose()

    mask = np.zeros( colorMap.shape, dtype = np.uint8 )
    cv2.drawContours( mask, [ contour ], 0, 1, -1 )

    return contour, mask

def findChangeOfColor( colorMap, color1, color2, mask = None, axis = 0, doFlip = False ):
    
    mask = mask if mask is not None else np.ones( colorMap.shape[:2] )

    marker = colorMap * mask
    marker[ np.logical_and( marker!=color1, marker!=color2 ) ] = np.max( marker ) + 100

    if doFlip:
        marker = np.flip(marker, axis = axis)

    diffMark = np.diff( marker, axis = axis )
    yEdge, xEdge = np.where( diffMark == color1 - color2 )

    pointClound = []
    for i in range( 0, colorMap.shape[ (axis+1) % 2 ] ):
        yCoor = yEdge[ xEdge == i ].astype( int )
        yCoor = np.vstack( ( np.ones( yCoor.shape, dtype = int ) * i, yCoor ) ).transpose()

        pointClound.append( yCoor )

    return pointClound
