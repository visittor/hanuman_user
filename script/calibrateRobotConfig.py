import numpy as np
import cv2
from scipy.optimize import minimize, LinearConstraint, BFGS, differential_evolution,SR1
import math

import optparse

import configobj

from utility.HanumanForwardKinematic import *

from utility.transformationModule import getInverseHomoMat, Project2Dto3D

ORIGINAL_CONFIG = getRobotConfiguration( )

def lossFunction_chessboard( robotConfig, data, camMat, distCoeffs, vis = False ):
	'''
	Loss function for minimizer. Project point in 3d coor to image plane then find
	and distance between projected point with actual image point.
	'''

	# robotConfig = list( robotConfig[:2] ) + list(ORIGINAL_CONFIG[2:-1]) + list( robotConfig[2:] )

	objPointList = data[ 'objPointList' ].copy()
	imgPointList = data[ 'imgPointList' ].copy()
	jsList = data[ 'jsList' ].copy()

	## Set robot dimension.
	setNewRobotConfiguration( *robotConfig )

	## Error
	sumError = 0

	k1 = distCoeffs[0]
	k2 = distCoeffs[1]
	k3 = distCoeffs[4] if len( distCoeffs ) >- 5 else 0.0
	p1 = distCoeffs[2]
	p2 = distCoeffs[3]
	i = 0
	## Loop for every points.
	for objP, imgP, js in zip( objPointList, imgPointList, jsList ):

		## Get transformation matrix .
		transformationMatrix = getMatrixForForwardKinematic( *js )
		invTransformationMatrix = getInverseHomoMat( transformationMatrix )
		
		imgP = imgP.reshape( -1, 2 )
		predObjP = Project2Dto3D( imgP, invTransformationMatrix, camMat)

		## Find error for each points.
		error = np.linalg.norm( predObjP - objP.reshape(-1,3), axis = 1 )
		error = np.sum( error ) / error.shape[0]
		sumError += error

		# print "pred image points : ", np.around(predImgP, decimals = 2)
		# print "actual image points : ", np.around(imgP.reshape(-1,2), decimals = 2)
		# print "-----------------------------"

		if vis:
			blank = np.zeros( (1000, 1000,3), dtype = np.uint8 )
			for pred, act in zip(predObjP, objP.reshape(-1,3)):
				xPred, yPred, _ = pred
				xPred = int(xPred*1000)
				yPred = int(yPred*1000) + 500
				cv2.circle( blank, (xPred, yPred), 3, (0,255,0), -1 )

				xAct, yAct, _ = act
				xAct = int( xAct*1000 )
				yAct = int( yAct*1000 ) + 500

				cv2.line( blank, (xPred, yPred), (xAct, yAct), (255,255,255), 1 )
				cv2.circle( blank, ( xAct, yAct), 3, (0,0,255), -1 )

			cv2.imshow( "test", blank )
			cv2.waitKey( 0 )
	print "Loss Chessboard: ", sumError / len( objPointList )
	## Find average error.
	return sumError / len( objPointList )

def lossFunction_line( robotConfig, data, camMat, distCoeffs, vis = False ):
	
	# assert A != 0 or B != 0

	imgPointList = data[ 'imgPointList' ]
	jsList = data[ 'jsList' ]

	setNewRobotConfiguration( *robotConfig )

	sumError = 0

	for imgP, js, coef in zip( imgPointList, jsList, data[ 'lineCoef' ] ):

		A, B, C = coef

		transformationMatrix = getMatrixForForwardKinematic( *js )
		invTransformationMatrix = getInverseHomoMat( transformationMatrix )

		imgP = imgP.reshape( -1, 2 )
		predObjP = Project2Dto3D( imgP, invTransformationMatrix, camMat )

		dist2Line = np.absolute( A*predObjP[:,0] + B*predObjP[:,1] + C ) / math.sqrt( A**2 + B**2 )
		# print dist2Line
		# print sum( dist2Line ) / len( dist2Line )
		# dist2Line = np.sum( dist2Line ) / len( dist2Line )
		dist2Line = max( dist2Line )

		sumError += dist2Line

		if vis:
			blank = np.zeros( (1000, 1000, 3), dtype = np.uint8 )

			if A == 0:
				x1, x2 = 0, 1000
				y1 = y2 = 100 * int(-C / B) + 500

			elif B == 0:
				x1 = x2 = 100 * int( -C / A ) + 500
				y1, y2 = 0, 1000

			else:
				x1 = 0
				y1 = 100 * int( -( A*x1 + C ) / B ) + 500

				x2 = 1000
				y2 = 100 * int( -( A*x2 + C ) / B ) + 500

			cv2.line( blank, (x1, y1), (x2, y2), (0,0,255), 1 )

			for pred in predObjP:

				x, y = pred[:2]
				x = int( 100 * x ) + 500
				y = int( 100 * y ) + 500

				cv2.circle( blank, (x, y), 3, (0,255,0), -1 )

			cv2.imshow( 'test', blank )
			cv2.waitKey( 0 )

	print "Loss Line: ", sumError / len( imgPointList )

	return sumError / len( imgPointList )

def lossFunction( robotConfig, dataChessboard, dataLine, camMat, distCoeffs, vis = False ):

	if dataChessboard is not None:
		loss_chessboard = lossFunction_chessboard( robotConfig, dataChessboard, camMat, distCoeffs, vis = vis )
	else:
		loss_chessboard = 0

	if dataLine is not None:
		loss_line = lossFunction_line( robotConfig, dataLine, camMat, distCoeffs, vis = vis )
	else:
		loss_line = 0

	print "Loss: ", loss_chessboard + loss_line
	print "\n"
	return loss_chessboard + loss_line

def main():
	parser = optparse.OptionParser()

	parser.add_option( '-p', type='string', action = 'store', dest = 'path_chessboard',
						help = 'path to object point, image point and pantilt pos.' )

	parser.add_option( '-l', type='string', action = 'store', dest = 'path_line',
						help = 'path to object point on field boundary.' )

	parser.add_option( '-s', type='string', action = 'store', dest = 'savePath',
						help = 'path for save intrinsic camera matrix and distortion coeff.')

	parser.add_option( '--camMat', type='string', action = 'store', dest = 'camMatPath',
					help = 'path to camera matrix.' )

	parser.add_option( '--resultPath', type='string', action = 'store',
						dest = 'resultPath', help = 'path for saving result.' )

	parser.add_option( '--visualize', action = 'store_true',
						dest = 'visualize', help = 'To only visualize error.')
 
	(options, args) = parser.parse_args()

	## Load image points, object point and joint state.
	data = np.load( options.path_chessboard )

	objPointList_chessboard = np.array( data[ 'objectPoints' ] )

	objPointList_chessboard = objPointList_chessboard.astype( np.float32 )
	imgPointList_chessboard = np.array( data[ 'imagePoints' ] )

	pantiltPosList_chessboard = np.array( data[ 'pantilt' ] )

	data = np.load( options.path_line )

	imgPointList_line = [np.vstack( pList )  for pList in data[ 'imagePoints' ]]
	pantiltPosList_line = np.array( data[ 'pantilt' ] )
	lineCoef = np.array( data[ 'lineCoef' ] )

	if options.camMatPath is not None:
		## Load camera matrix.
		camera_prop = np.load( options.camMatPath )
		cameraMatrix = camera_prop[ 'cameraMatrix' ]
		distCoeffs = camera_prop[ 'distCoeffs' ]
		# roi = camera_prop[ 'roi' ]

	else:
		## Re-calculate camera matrix
		retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera( objPointList_chessboard, 
														imgPointList_chessboard, (640,480), None, None )
		# cameraMatrix = np.eye( 3 )
		# distCoeffs = np.array([[0, 0, 0, 0, 0]])
		# newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix( cameraMatrix,
		# 													distCoeffs,
		# 													(640,480), 
		# 													0,
		# 													(640,480))

		distCoeffs = distCoeffs[0]

		if options.savePath is not None:
			np.savez( options.savePath,
						cameraMatrix = cameraMatrix,
						distCoeffs = distCoeffs, )
	if options.resultPath is not None:

		camProp = { 'fx' : cameraMatrix[0,0], 'fy' : cameraMatrix[1,1],
					'cx' : cameraMatrix[0,2], 'cy' : cameraMatrix[1,2] }

		try:
			config = configobj.ConfigObj( options.resultPath )
		except Exception:
			config = configobj.ConfigObj()
			config.filename = options.resultPath
		print distCoeffs
		config['CameraParameters']['fx'] = cameraMatrix[0,0]
		config['CameraParameters']['fy'] = cameraMatrix[1,1]
		config['CameraParameters']['cx'] = cameraMatrix[0,2]
		config['CameraParameters']['cy'] = cameraMatrix[1,2]
		config['CameraParameters']['k1'] = distCoeffs[ 0 ]
		config['CameraParameters']['k2'] = distCoeffs[ 1 ]
		config['CameraParameters']['k3'] = distCoeffs[ 4 ] if len( distCoeffs ) >= 5 else 0.0
		config['CameraParameters']['p1'] = distCoeffs[ 2 ]
		config['CameraParameters']['p2'] = distCoeffs[ 3 ]

		config.write( )

	if options.visualize:
		loadDimensionFromConfig( options.resultPath )
		x = getRobotConfiguration( )
		print x
		dataChessboard = { 	'objPointList' : objPointList_chessboard,
						'imgPointList' : imgPointList_chessboard,
						'jsList' : pantiltPosList_chessboard }

		dataLine = { 	'imgPointList' : imgPointList_line,
						'jsList' : pantiltPosList_line,
						'lineCoef' : lineCoef }

		args = ( dataChessboard, dataLine, cameraMatrix, distCoeffs )

		lossFunction( x, *args, vis = True )

		return 

	# cameraMatrix[0,0] = 625.44622803
	# cameraMatrix[1,1] = 579.21398926
	# cameraMatrix[0,2] = 330.76631757
	# cameraMatrix[1,2] = 161.34627042

	###########################################################################
	## Minimizer Part

	## Our initial guess.
	initialGuess = np.array( [ 0.46,
								0.0,
								0.02,
								0.03,
								0.07,
								0.02,
								0.005,
								5.0,
								0.0 ] )
	# objPointList = objPointList[ : len( objPointList )/2]
	# imgPointList = imgPointList[ : len( imgPointList )/2]
	# pantiltPosList = pantiltPosList[ : len( pantiltPosList )/2]

	dataChessboard = { 	'objPointList' : objPointList_chessboard,
						'imgPointList' : imgPointList_chessboard,
						'jsList' : pantiltPosList_chessboard }

	dataLine = { 	'imgPointList' : imgPointList_line,
					'jsList' : pantiltPosList_line,
					'lineCoef' : lineCoef }

	args = ( dataChessboard, None, cameraMatrix, distCoeffs )
	method = "trust-constr"
	jac = "3-point"
	hess = BFGS()
	bounds = [ (0.2,0.6), 
				(-0.1	, 0.5), 
				(0.005, 0.05), 
				(0.005, 0.05), 
				(0.035, 0.2), 
				(0.0, 0.05),
				(-0.02, 0.02 ), 
				(-90, 90),
				(-90, 90) ]

	constr = LinearConstraint( np.eye(len(bounds)), [i[0] for i in bounds], [i[1] for i in bounds] )

	resultTrustConstr = minimize( lossFunction, initialGuess,
						args = args,
						method = method,
						jac = jac,
						hess = hess,
						bounds = bounds,
						constraints = constr
						 )

	# resultEvo = differential_evolution(lossFunction, bounds, 
	# 								args = args, 
	# 								)
	args = ( dataChessboard, dataLine, cameraMatrix, distCoeffs )
	x = resultTrustConstr.x
	lossFunction( x.copy(), *args, vis = True )

	print "Loss Trust-constr: ", resultTrustConstr.fun
	print "Result Trust-constr: ", resultTrustConstr.x
	print cameraMatrix

	# print "Loss Evo: ", resultEvo.fun
	# print "Result Evo: ", resultEvo.x

	# finalConfig = resultEvo.x if resultEvo.fun < resultTrustConstr.fun else resultTrustConstr.x
	finalConfig = x

	# finalConfig = list(finalConfig[:2]) + list(ORIGINAL_CONFIG[2:-1]) + list(finalConfig[2:]) 

	setNewRobotConfiguration( *finalConfig )

	if options.resultPath is not None:
		saveDimension( options.resultPath )

if __name__ == '__main__':
	main()