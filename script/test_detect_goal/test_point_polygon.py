#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

VERSIONNUMBER = 'v1.0'
PROGRAM_DESCRIPTION = "EIEI"

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os

import optparse


########################################################
#
#	LOCAL IMPORTS
#

import cv2
import numpy as np

########################################################
#
#	Standard globals
#
NUM_REQUIRE_ARGUMENT = 0

########################################################
#
#	Program specific globals
#

########################################################
#
#	Helper functions
#

def createFourPoints( origin, width, height ):
	'''	createFourPoints function
	'''

	#		topLeft (origin) --------- topRight
	#				|					   |
	#				|					   |
	#				|					   |
	#	  	    bottomLeft	 ---------  bottomRight (final position) 

	#	NOTE: Arrange by bottomLeft, topLeft

	#	topLeft x0, y0 -> origin
	topLeft = origin

	#	topRight x0+w, y0
	topRight = ( origin[0] + width, origin[1] )

	#	bottomLeft x0, y0+h
	bottomLeft = ( origin[0], origin[1] + height )

	#	bottomRight x0+w,y0+h
	bottomRight = ( origin[0]+width, origin[1]+height )

	fourPointList = [ bottomLeft, topLeft, topRight, bottomRight ]
	# fourPointList = [ topLeft, topRight, bottomLeft, bottomRight ]

	return fourPointList
########################################################
#
#	Class definitions
#

########################################################
#
#	Function bodies
#

########################################################
#
#	main
#	
def main():
	
	#	define usage of programing
	programUsage = "python %prog arg [option] {} ".format( '' ) + str( VERSIONNUMBER ) + ', Copyright (C) 2019 FIBO/KMUTT'

	#	initial parser instance
	parser = optparse.OptionParser( usage = programUsage, description=PROGRAM_DESCRIPTION )

	#	add option of main script
	parser.add_option( "-o", "--myOption", dest = "myOption",
						help = "Specify option document here." )

	#	add option
	( options, args ) = parser.parse_args()

	#	check number of argument from NUM_REQUIRE_ARGUMENT
	if len( args ) != NUM_REQUIRE_ARGUMENT:	
		
		#	raise error from parser
		parser.error( "require {} argument(s)".format( NUM_REQUIRE_ARGUMENT ) )
	

	#########################################################
	#
	#		get option and argument
	#

	img = np.zeros( (400, 400, 3), dtype=np.uint8 )

	width = 100
	height = 100

	#
	#	Generate fourpoints
	#
	fourPointList = createFourPoints( (100,100), width, height )

	#	Change to contour for opencv
	rectContour = np.array( fourPointList ).reshape( -1, 1, 2 )

	#	Try to create some point to test
	p1 = ( 120, 120 )
	p2 = ( 180, 160 )
	p3 = ( 150, 150 )
	p4 = ( 150, 170 )
	p5 = ( 150, 250 )
	pList = [ p1, p2, p3, p4, p5 ]

	rawScoreList = list()
	scoreList = list()

	#	Test poly gon
	for p in pList:

		score = cv2.pointPolygonTest( rectContour, p, True )
		rawScoreList.append( score )
		score /= ( min( width, height ) / 2.0 )
		scoreList.append( score )

	print "Contours"
	for p in rectContour:
		cv2.putText( img, "{}".format( p ), ( p[ 0, 0 ], p[ 0, 1 ] ), cv2.FONT_HERSHEY_SIMPLEX, 0.3, ( 255, 255, 255 ), 1, cv2.LINE_AA )

	print "Point list : {}".format( pList )
	print "Raw score list : {}".format( rawScoreList )
	print "Score List : {}".format( scoreList )
				
	#	Try to plot in image
	for p in pList:
		cv2.circle( img, p, 5, ( 0, 0, 255 ), -1 )
		cv2.putText( img, "{}".format( p ), p, cv2.FONT_HERSHEY_SIMPLEX, 0.4, ( 255, 255, 255 ), 1, cv2.LINE_AA )

	#	Visualize rectangle contours
	cv2.drawContours( img, [rectContour], 0, ( 255, 0, 0 ), 2 )

	cv2.imshow( 'img', img )
	cv2.waitKey( 0 )
	cv2.destroyAllWindows() 

########################################################
#
#	call main
#

if __name__=='__main__':
	main()

