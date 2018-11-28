import numpy as np
import cv2

from newbie_hanuman.msg import ColorDef

def colorSegmentation( img, colorDef ):

	marker = np.zeros( img.shape, dtype = np.int32 )
	for color in colorDef:
		lower = np.array( [ color.H_min, color.S_min, color.V_min ] )
		upper = np.array( [ color.H_max, color.S_max, color.V_max ] )

		colorMask = cv2.inRange( img, lower, upper )
		marker[ colorMask != 0 ] = color.ID 
	
	return marker

def createColorDefFromDict( colorConfigList ):
	colorDefList = []
	for colorConfig in colorConfigList:
		colorDef = ColorDef( 
							ID = colorConfig["ID"],
							Name = colorConfig["Name"],
							RenderColor_RGB = colorConfig["RenderColor_RGB"],
							# MinArea_pixels = colorConfig["MinArea_pixels"],
							H_max = colorConfig["H_max"],
							H_min = colorConfig["H_min"],
							S_max = colorConfig["S_max"],
							S_min = colorConfig["S_min"],
							V_max = colorConfig["V_max"],
							V_min = colorConfig["V_min"],
		 					)
		colorDefList.append( colorDef )
	
	return colorDefList

def waterShed( img, marker ):
	colorSeg = cv2.watershed( img, marker )
	return colorSeg