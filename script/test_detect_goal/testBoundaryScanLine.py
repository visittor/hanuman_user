import sys
import os

import optparse

import cv2
import numpy as np

from scanLine2 import findBoundary, findLinearEqOfFieldBoundary

import configobj

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
		
		lower = np.array( [ int( colorDef[ colorKey ][ "H_Min" ] ), int( colorDef[ colorKey ][ "S_Min" ] ), int( colorDef[ colorKey ][ "V_Min" ] ) ] )
		upper = np.array( [ int( colorDef[ colorKey ][ "H_Max" ] ), int( colorDef[ colorKey ][ "S_Max" ] ), int( colorDef[ colorKey ][ "V_Max" ] ) ] ) 

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
	print colorDef
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

	while True:

		ret, frame = cap.read( )

		if not ret:
			cap.set( cv2.CAP_PROP_POS_FRAME, 0 )

			continue

		blurImage = cv2.GaussianBlur( frame, (5,5), 0 )

		marker = colorSegmentation( blurImage, colorDef )
		# marker = waterShed( blurImage, marker )

		fieldContour, fieldMask = findBoundary( marker, options.colorGreen )

		fieldBoundary = findLinearEqOfFieldBoundary( fieldContour )

		p_list = []

		waitTime = 1

		for m, c, x0, xf in fieldBoundary:

			c += 5

			y0 = ( m * x0 ) + c
			yf = ( m * xf ) + c

			p_list.append( (int(x0), int(y0), int(xf), int(yf)) )

		changeFG2BG = []
		changeBG2FG = []
		FG = []
		BG = []

		for x0, y0, xf, yf in p_list:

			cv2.line( frame, (x0,y0), (xf,yf), (0,255,255), 1 )
			# cv2.circle( frame, (x0,y0), 5, (0,0,255), -1 )

			# if xf < 0:
			# 	print p_list
			# 	print fieldBoundary
			# 	waitTime = 0
			changeFG2BG_, changeBG2FG_, FG_, BG_ = raycast_bresenham( x0, y0, xf, yf, marker, options.colorGreen, options.colorWhite )

			changeFG2BG.extend( changeFG2BG_ )
			changeBG2FG.extend( changeBG2FG_ )
			FG.extend( FG_ )
			BG.extend( BG_ )

		for x,y in BG:
			cv2.circle( frame, (x,y), 2, (0,255,0), -1 )

		for x,y in FG:
			cv2.circle( frame, (x,y), 2, (0,0,0), -1 )

		for x,y in changeFG2BG:
			cv2.circle( frame, (x,y), 3, (0,0,255), -1 )

		for x,y in changeBG2FG:
			cv2.circle( frame, (x,y), 3, (255,0,0), -1 )

		renderImg = renderColor( marker, colorDef )

		cv2.imshow( 'frame', frame )

		k = cv2.waitKey( 0 )

		if k == ord( 'q' ):
			break

	cv2.destroyAllWindows( )

if __name__ == '__main__':

	main( )