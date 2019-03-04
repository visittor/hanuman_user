import numpy as np
import cv2

from newbie_hanuman.msg import ColorDef

def colorSegmentation( img, colorDef ):

	hsvImage = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

	marker = np.zeros( ( img.shape[ 0 ], img.shape[ 1 ] ), dtype = np.int32 )
	for color in colorDef:
		lower = np.array( [ color.H_min, color.S_min, color.V_min ] )
		upper = np.array( [ color.H_max, color.S_max, color.V_max ] )

		colorMask = cv2.inRange( hsvImage, lower, upper )
		marker[ colorMask != 0 ] = color.ID 
	
	return marker

def createColorDefFromDict( colorConfigList ):
	colorDefList = []
	for colorConfig in colorConfigList:
		colorDef = ColorDef( 
							ID = int( colorConfig["ID"] ),
							Name = colorConfig["Name"],
							RenderColor_RGB = colorConfig["RenderColor_RGB"],
							# MinArea_pixels = colorConfig["MinArea_pixels"],
							H_max = int( colorConfig["H_max"] ),
							H_min = int( colorConfig["H_min"] ),
							S_max = int( colorConfig["S_max"] ),
							S_min = int( colorConfig["S_min"] ),
							V_max = int( colorConfig["V_max"] ),
							V_min = int( colorConfig["V_min"] ),
		 					)
		colorDefList.append( colorDef )
	
	return colorDefList

def waterShed( img, marker ):
	colorSeg = cv2.watershed( img, marker )
	return colorSeg