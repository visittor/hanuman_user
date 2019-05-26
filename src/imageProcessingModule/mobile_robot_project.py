import numpy as np
import cv2

def findBiggestContour( mask ):

	_, contours, _ = cv2.findContours( mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

	largestArea = -1
	selectedCntCenter = []

	for cnt in contours:
		area = cv2.contourArea(cnt)

		# epsilon = 0.01*cv2.arcLength(cnt,True)
		# approx = cv2.approxPolyDP(cnt, epsilon, True)
		# print len( approx )
		# if (area > largestArea or largestArea < 0) and len( cnt ) > 8 :
		if len( cnt ) > 8:
			x,y,w,h = cv2.boundingRect(cnt)

			selectedCntCenter.append( [ x+(w/2), y+h ] ) 

	return selectedCntCenter

def findPole( colorMap, orangeID, blueID, yellowID, magentaID, mask = None ):

	if mask is None:
		mask = np.ones( colorMap.shape )

	pole = {}

	maskorange = (colorMap == orangeID) * mask
	cntCenter = findBiggestContour( maskorange )

	pole['orange'] = cntCenter


	maskblue = (colorMap == blueID) * mask
	cntCenter = findBiggestContour( maskblue )

	pole['blue'] = cntCenter

	maskyellow = (colorMap == yellowID) * mask
	cntCenter = findBiggestContour( maskyellow )

	pole['yellow'] = cntCenter

	maskmagenta = (colorMap == magentaID) * mask
	cntCenter = findBiggestContour( maskmagenta )

	pole['magenta'] = cntCenter

	return pole