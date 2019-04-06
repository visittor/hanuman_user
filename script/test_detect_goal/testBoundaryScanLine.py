import sys
import os

import optparse

import cv2
import numpy as np

from scanLine2 import findBoundary, findLinearEqOfFieldBoundary
from scanLine2 import findNewLineFromRansac

import configobj

from scipy import spatial

def readConfig( pathConfig ):
	"""
	Read config and get dictionary
	"""
	
	#   create instance of config object
	config = configobj.ConfigObj( pathConfig )

	#	get color definitions
	colorDefinition = config[ 'ColorDefinitions' ]

	return colorDefinition

def colorSegmentation( img, colorDef ):
	"""
	Find the marker that watershed want
	"""

	imgHSV = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

	marker = np.zeros( ( img.shape[ 0 ], img.shape[ 1 ] ), dtype = np.int32 )
	
	for colorKey in colorDef.keys():
		
		lower = np.array( [ int( colorDef[ colorKey ][ "H_min" ] ), int( colorDef[ colorKey ][ "S_min" ] ), int( colorDef[ colorKey ][ "V_min" ] ) ] )
		upper = np.array( [ int( colorDef[ colorKey ][ "H_max" ] ), int( colorDef[ colorKey ][ "S_max" ] ), int( colorDef[ colorKey ][ "V_max" ] ) ] ) 

		colorMask = cv2.inRange( imgHSV, lower, upper )
		marker[ colorMask != 0 ] = int( colorDef[ colorKey ][ "ID" ] )
	
	return marker

def waterShed( img, marker ):
	"""
	Water shred function (from opencv)
	"""
	colorSeg = cv2.watershed( img, marker )
	return colorSeg

def renderColor( marker, colorDef ):
	#print colorDef
	img = np.zeros( (marker.shape[0], marker.shape[1], 3), dtype = np.uint8 )
	for colorKey in colorDef.keys( ):

		img[marker == int(colorDef[colorKey]['ID']),:] = eval( colorDef[colorKey]['RenderColor_RGB'])

	return img

def raycast_bresenham( x0, y0, x1, y1, colorMap, colorBG, colorFG ):

	changeBG2FG = []
	changeFG2BG = []
	BG = []
	FG = []

	steep = 0

	x = x0
	y = y0

	dx = abs( x1 - x )
	sx = 1 if x1 - x > 0 else -1

	dy = abs( y1 - y )
	sy = 1 if y1 - y > 0 else -1

	if dy > dx:
		steep = 1
		x,y = y,x
		dx,dy = dy,dx
		sx,sy = sy,sx

	d = ( 2 * dy ) - dx

	isBG = True if colorMap[ y0, x0 ] == colorBG else False

	for i in range( 0, dx ):

		xx = y if steep else x
		yy = x if steep else y

		if isBG and colorMap[ yy, xx ] != colorBG:
			changeBG2FG.append( (xx, yy) )

		if not isBG and colorMap[ yy, xx ] == colorBG:
			changeFG2BG.append( (xx, yy) )

		isBG = True if colorMap[ yy, xx ] == colorBG else False

		if isBG:
			BG.append( (xx,yy) )
		else:
			FG.append( (xx,yy) )

		while d >= 0:
			y = y + sy
			d = d - ( 2 * dx )

		x = x + sx
		d = d + ( 2 * dy )

	return changeFG2BG, changeBG2FG, FG, BG

def main( ):

	usage = "python %prog vidoeInputPath colorConfigPath"
	parser = optparse.OptionParser( usage = usage )

	parser.add_option( '--colorGreen', dest = 'colorGreen',
						type = 'int', 
						help = 'Color ID for green color.',
						default = 1 )
	parser.add_option( '--colorWhite', dest = 'colorWhite', 
						type = 'int',
						help = 'Color ID for white color.',
						default = 0 )

	options, args = parser.parse_args( )

	assert len( args ) == 2, "This program take exacly 1 argument ({} given).".format( len(args) )
	
	videoPath = args[0]
	colorConfigPath = args[1] 

	colorDef = readConfig( colorConfigPath )

	cap	= cv2.VideoCapture( videoPath )
	#cap = cv2.VideoCapture( 1 )
	
	frameIdx = 0 

	while True:

		ret, frame = cap.read( )
		
		cap.set( cv2.CAP_PROP_POS_FRAMES, frameIdx )
		
		#	get image width and height
		imageWidth, imageHeight = frame.shape[ 1 ], frame.shape[ 0 ]

		if not ret:
			cap.set( cv2.CAP_PROP_POS_FRAME, 0 )

			continue

		blurImage = cv2.GaussianBlur( frame, (5,5), 0 )

		marker = colorSegmentation( blurImage, colorDef )
		marker = waterShed( blurImage, marker )

		fieldContour, fieldMask = findBoundary( marker, 2 )

		#fieldBoundary = findLinearEqOfFieldBoundary( fieldContour )
		
		#	find new contour 
		ransacContours = findNewLineFromRansac( fieldContour, imageWidth, imageHeight )
		
		ransacContours1 = ransacContours.copy()
		ransacContours2 = ransacContours.copy()
		ransacContours3 = ransacContours.copy()
		
		ransacContours1[ :, :, 1 ] = ransacContours1[ :, :, 1 ] + 5
		ransacContours2[ :, :, 1 ] = ransacContours2[ :, :, 1 ] + 10
		ransacContours3[ :, :, 1 ] = ransacContours3[ :, :, 1 ] + 15
		
		previousPoint1 = ransacContours1[ 1 ]
		previousPoint2 = ransacContours2[ 1 ]
		previousPoint3 = ransacContours3[ 1 ]
		
		changeList1 = list()
		changeList2 = list()
		changeList3 = list()
			
			
		#	cut origin and final position
		for point in ransacContours1[ 1:-1 ]:
			
			if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] != marker[ previousPoint1[ 0 ][ 1 ], previousPoint1[ 0 ][ 0 ] ]:
				
				if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 2 or marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 5:
					#isChange = True
					changeList1.append( point )
			
			previousPoint1 = point
			
			
		for point in ransacContours2[ 1:-1 ]:
			
			if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] != marker[ previousPoint2[ 0 ][ 1 ], previousPoint2[ 0 ][ 0 ] ]:
				
				if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 2 or marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 5:
					#isChange = True
					changeList2.append( point )
			
			previousPoint2 = point

		for point in ransacContours3[ 1:-1 ]:
			
			if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] != marker[ previousPoint3[ 0 ][ 1 ], previousPoint3[ 0 ][ 0 ] ]:
				
				if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 2 or marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 5:
					#isChange = True
					changeList3.append( point )
			
			previousPoint3 = point

		
		
		changeArray2 = np.array( changeList2 ).reshape( -1, 2 )
		changeArray3 = np.array( changeList3 ).reshape( -1, 2 )
		
		print "************** ChangeArray *****************"
		print changeArray2
		print changeArray3
		
		if len( changeArray2 ) != 0 and len( changeArray3 ) != 0:
		
			distanceMatrix = spatial.distance.cdist( changeArray2, changeArray3 ) 

			minIdxAxis_0 = np.argmin( distanceMatrix, axis = 0 )
			minIdxAxis_1 = np.argmin( distanceMatrix, axis = 1 )
			print "************** Idx *****************"
			print minIdxAxis_0
			print minIdxAxis_1

			print "************** Distance *****************"
			print distanceMatrix

	#		for originPoints in changeList1:
	#			x = originPoints[ 0 ][ 0 ]
	#			y = originPoints[ 0 ][ 1 ]
	#			
	#			cv2.circle( frame, ( x, y ), 3, ( 0, 0, 255 ), -1 )

	#		for originPoints in changeList2:
	#			x = originPoints[ 0 ][ 0 ]
	#			y = originPoints[ 0 ][ 1 ]
	#			
	#			cv2.circle( frame, ( x, y ), 3, ( 0, 0, 255 ), -1 )
	#		
	#		for originPoints in changeList3:
	#			x = originPoints[ 0 ][ 0 ]
	#			y = originPoints[ 0 ][ 1 ]
	#			
	#			cv2.circle( frame, ( x, y ), 3, ( 0, 0, 255 ), -1 )
				#print marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 2, marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 5

			goalPointList = list()

			for i, j in zip( minIdxAxis_0, minIdxAxis_1 ):
				if i == j and 4.5 < distanceMatrix[ i, j ] < 6.5:
					x1, y1 = changeList2[ i ][ 0 ][ 0 ], changeList2[ i ][ 0 ][ 1 ]
					x2, y2 = changeList3[ j ][ 0 ][ 0 ], changeList3[ j ][ 0 ][ 1 ]

					goalPointList.append( ( x1, y1 ) )

					cv2.circle( frame, ( x1, y1 ), 3, ( 0, 255, 0 ), -1 )
					cv2.circle( frame, ( x2, y2 ), 3, ( 0, 0, 255 ), -1 )
			print goalPointList		
			if len( goalPointList ) == 4:
				
				print "found goal on frame : {}".format( cap.get( cv2.CAP_PROP_POS_FRAMES ) )
				
				distanceMatrix2 = spatial.distance.cdist( goalPointList, goalPointList )
				
#				print distanceMatrix2
#				print goalPointList
				
#				minIdxAxisGoal_0 = np.argmin( distanceMatrix2[ np.nonzero( distanceMatrix2 ) ], axis = 0 )
#				minIdxAxisGoal_1 = np.argmin( distanceMatrix2[ np.nonzero( distanceMatrix2 ) ], axis = 1 )
#				
#				print "idx : {}, {}".format( minIdxAxisGoal_0, minIdxAxisGoal_1 )
				
	#		waitTime = 1
	#
	#		for m, c, x0, xf in fieldBoundary:
	#
	#			c += 5
	#
	#			y0 = ( m * x0 ) + c
	#			yf = ( m * xf ) + c
	#
	#			p_list.append( (int(x0), int(y0), int(xf), int(yf)) )
	#
	#		changeFG2BG = []
	#		changeBG2FG = []
	#		FG = []
	#		BG = []
	#
	#		for x0, y0, xf, yf in p_list:
	#			print x0, y0, xf, yf
	#
	#			cv2.line( frame, (x0,y0), (xf,yf), (0,255,255), 1 )

				# cv2.circle( frame, (x0,y0), 5, (0,0,255), -1 )

				# if xf < 0:
				# 	print p_list
				# 	print fieldBoundary
				# 	waitTime = 0
	#			changeFG2BG_, changeBG2FG_, FG_, BG_ = raycast_bresenham( x0, y0, xf, yf, marker, options.colorGreen, options.colorWhite )
	#
	#			changeFG2BG.extend( changeFG2BG_ )
	#			changeBG2FG.extend( changeBG2FG_ )
	#			FG.extend( FG_ )
	#			BG.extend( BG_ )

	#		for x,y in BG:
	#			cv2.circle( frame, (x,y), 2, (0,255,0), -1 )
	#
	#		for x,y in FG:
	#			cv2.circle( frame, (x,y), 2, (0,0,0), -1 )
	#
	#		for x,y in changeFG2BG:
	#			cv2.circle( frame, (x,y), 3, (0,0,255), -1 )
	#
	#		for x,y in changeBG2FG:
	#			cv2.circle( frame, (x,y), 3, (255,0,0), -1 )

		cv2.drawContours( frame, [ ransacContours ], 0, ( 255, 0, 0 ), 1 ) 

		renderImg = renderColor( marker, colorDef )

		cv2.imshow( 'frame', frame )
		cv2.imshow( 'rendered', renderImg )

		k = cv2.waitKey( 1 )

		if k == ord( 'q' ):
			break
		
		if k == ord( 'a' ):
			frameIdx -= 1
			
		if k == ord( 'd' ):
			frameIdx += 1

	cv2.destroyAllWindows( )

if __name__ == '__main__':

	main( )
