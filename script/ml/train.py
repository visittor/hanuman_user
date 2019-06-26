#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

VERSIONNUMBER = 'v1.0'
PROGRAM_DESCRIPTION = "Training hog+svm"

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os

import optparse

import time

########################################################
#
#	LOCAL IMPORTS
#

import numpy as np
import cv2

from feature_extraction import HOG, ColorHistrogram

########################################################
#
#	Standard globals
#
NUM_REQUIRE_ARGUMENT = 2

########################################################
#
#	Program specific globals
#

########################################################
#
#	Helper functions
#

def buildSample( positiveData, negativeData, positiveLabel = 1, targetLabel = 0, isRowSample = True ):

	if isRowSample:
		positiveTarget = np.ones( positiveData.shape[ 0 ] ).astype( np.int64 )
		negativeTarget = np.ones( negativeData.shape[ 0 ] ).astype( np.int64 )
	else:
		positiveTarget = np.ones( positiveData.shape[ 1 ] ).astype( np.int64 )
		negativeTarget = np.ones( negativeData.shape[ 1 ] ).astype( np.int64 )
	
	positiveTarget = positiveTarget * positiveLabel
	negativeTarget = negativeTarget * targetLabel

	if isRowSample:
		data = np.vstack( ( positiveData, negativeData ) )
	else:
		data = np.hstack( ( positiveData, negativeData ) )

	target = np.hstack( ( positiveTarget, negativeTarget ) )

	#	Shuffle
	indexToShuffle = np.arange( target.size )
	np.random.shuffle( indexToShuffle )

	newData = data[ indexToShuffle ]
	newTarget = target[ indexToShuffle ]

	return newData, newTarget


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
	programUsage = "python %prog arg [option] {} ".format( "[positiveSampleDir] [negativeSampleDir]" ) + str( VERSIONNUMBER ) + ', Copyright (C) 2019 FIBO/KMUTT'

	#	initial parser instance
	parser = optparse.OptionParser( usage = programUsage, description=PROGRAM_DESCRIPTION )

	#	add option of main script
	parser.add_option( "--savePath", dest = "savePath", type = "string",
						help = "Specify option document here.", default = "model.yaml" )

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
	positiveSamplePathDir = args[ 0 ]
	negativeSamplePathDir = args[ 1 ]

	savePathStr = options.savePath


	#   Get to hog
	hogPositive = ColorHistrogram( positiveSamplePathDir, winSize=( 64, 64 ), blockSize=( 16, 16 ), cellSize = ( 8, 8 ), blockStride=( 8, 8 ) )
	hogNegative = ColorHistrogram( negativeSamplePathDir, winSize=( 64, 64 ), blockSize=( 16, 16 ), cellSize = ( 8, 8 ), blockStride=( 8, 8 ) )

	print "\nExtract positive..."
	positiveFeature = hogPositive.doExtract(  )
	print "Done. Positive feature shape {}".format( positiveFeature.shape )

	print "Extract negative..."
	negativeFeature = hogNegative.doExtract(  )
	print "Done. Negative feature shape {}".format( negativeFeature.shape )


	print "\nMake sample..."
	#   Make sample
	data, target = buildSample( positiveFeature, negativeFeature )
	print "Shape of data : {}, target data : {}".format( data.shape, target.shape )

	print "\nInit svm..."
	svm = cv2.ml.SVM_create()

	svm.setTermCriteria( ( cv2.TERM_CRITERIA_MAX_ITER, 1000, 1e-6 ) )
	svm.setKernel( cv2.ml.SVM_RBF )
	svm.setType( cv2.ml.SVM_C_SVC )
	print "Done"

	print "\nTrain..."
	startTime = time.time()
	svm.trainAuto( data, cv2.ml.ROW_SAMPLE, target )
	print "Done, usage : {} second".format( time.time() - startTime )

	print "\nSave model..."
	svm.save( savePathStr )
	print "Done, save as {}".format( savePathStr )
########################################################
#
#	call main
#

if __name__=='__main__':
	main()

