#!/usr/bin/env python
#
# Copyright (C) 2018  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

VERSIONNUMBER = 'v1.0'
PROGRAM_DESCRIPTION = "Test region list and data struct "

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

from visionModule.Nasrun.region import BoundingBoxList

########################################################
#
#	Standard globals
#
NUM_REQUIRE_ARGUMENT = 0

TEST_IMAGE_PATH = 'testImage.jpg'

########################################################
#
#	Program specific globals
#

########################################################
#
#	Helper functions
#

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
	programUsage = "python %prog arg [option] " + str( VERSIONNUMBER ) + ', Copyright (C) 2018 FIBO/KMUTT'

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

	img = cv2.imread( TEST_IMAGE_PATH )[ 10 : 500, 10 : 450 ]
	testImage = cv2.imread( 'testMask.jpg' )
	ret, threshImage = cv2.threshold( img, 127, 255, cv2.THRESH_BINARY_INV )
	ret, threshTestImage = cv2.threshold( testImage, 127, 255, cv2.THRESH_BINARY_INV )
	grayImage = cv2.cvtColor( threshImage, cv2.COLOR_BGR2GRAY )
	grayTestImage = cv2.cvtColor( testImage, cv2.COLOR_BGR2GRAY )

	#	initial bounding list
	boundingList = BoundingBoxList( 20, 10 )
	boundingList_2 = BoundingBoxList( 20, 10 )

	#	get contours
	contours = cv2.findContours( grayImage, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]
	contoursTest = cv2.findContours( grayTestImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )[ 1 ]

	#	get bounding box
	boundingList.getBoundingBox( img, contours )
	boundingList_2.getBoundingBox( threshTestImage, contoursTest )

	for cnt in contours:

		x, y, w, h = cv2.boundingRect( cnt )

		cv2.rectangle( img, ( x, y ), ( x + w, y + h ), ( 255, 0, 0 ), 2  )

	for bouningBox in boundingList.boundingBoxList:

		cv2.rectangle( img, bouningBox.topLeftPositionTuple, bouningBox.bottomRightPositionTuple, ( 0, 255, 0 ), 2 )

	for bouningBox in boundingList_2.boundingBoxList:

		cv2.rectangle( threshTestImage, bouningBox.topLeftPositionTuple, bouningBox.bottomRightPositionTuple, ( 0, 255, 0 ), 2 )

	#	clear bounding list
	boundingList.clearBoundingBoxList()

	cv2.imshow( "image", img )
	cv2.imshow( "haha", threshTestImage )
	cv2.waitKey( 0 )
	cv2.destroyAllWindows()

########################################################
#
#	call main
#

if __name__=='__main__':
	main()

