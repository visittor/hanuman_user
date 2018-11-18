import numpy as np
import cv2

def colorSegmentation( img, colorDef ):

	marker = np.zeros( img.shape, dtype = np.int32 )
	for color in colorDef:
		lower = np.array( [ color.H_min, color.S_min, color.V_min ] )
		upper = np.array( [ color.H_max, color.S_max, color.V_max ] )

		colorMask = cv2.inRange( img, lower, upper )
		marker[ colorMask != 0 ] = color.ID 
	
	return marker

def waterShed( img, marker ):
	colorSeg = cv2.watershed( img, marker )
	return colorSeg