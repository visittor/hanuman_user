import numpy as np
import cv2
from scipy.optimize import minimize, LinearConstraint, BFGS, differential_evolution,SR1
import math

import optparse

import configobj

from utility.HanumanForwardKinematic import *

from utility.transformationModule import getInverseHomoMat

ORIGINAL_CONFIG = getRobotConfiguration( )

def lossFunction( robotConfig, objPointList, imgPointList, jsList, camMat, distCoeffs, vis = False ):
	'''
	Loss function for minimizer. Project point in 3d coor to image plane then find
	and distance between projected point with actual image point.
	'''

	# robotConfig = list( robotConfig[:2] ) + list(ORIGINAL_CONFIG[2:-1]) + list( robotConfig[2:] )

	## Set robot dimension.
	setNewRobotConfiguration( *robotConfig )

	## Error
	sumError = 0

	k1 = distCoeffs[0]
	k2 = distCoeffs[1]
	k3 = distCoeffs[4] if len( distCoeffs ) >- 5 else 0.0
	p1 = distCoeffs[2]
	p2 = distCoeffs[3]

	## Loop for every points.
	for objP, imgP, js in zip( objPointList, imgPointList, jsList ):

		## Get transformation matrix .
		transformationMatrix = getMatrixForForwardKinematic( *js )
		invTransformationMatrix = getInverseHomoMat( transformationMatrix )
		
		## Print this. To indicate that program is still running.
		# print 'HMat', transformationMatrix

		## Get homogenouse point
		homoObjP = np.vstack( (objP.T, np.ones( (objP.shape[0],) ) ) )
 
		## Project point to image plane.
		predObjP_cam = np.matmul( invTransformationMatrix[:3,:], homoObjP )
		predObjP_cam /= predObjP_cam[2]

		# r = predObjP_cam[0,:]**2 + predObjP_cam[1,:]**2

		# predObjP_cam[0,:] = predObjP_cam[0,:]*(1+(k1*r**2)+(k2*r**4)+(k3*r**6)) + 2*p1*predObjP_cam[0,:]*predObjP_cam[1,:] + p2*(r**2 + 2*predObjP_cam[0,:]**2)
		# predObjP_cam[1,:] = predObjP_cam[1,:]*(1+(k1*r**2)+(k2*r**4)+(k3*r**6)) + 2*p2*predObjP_cam[0,:]*predObjP_cam[1,:] + p1*(r**2 + 2*predObjP_cam[1,:]**2)

		predImgP = np.matmul( camMat, predObjP_cam )
		predImgP = predImgP[:-1]

		predImgP = predImgP.T

		## Find error for each points.
		error = np.linalg.norm( predImgP - imgP.reshape(-1,2), axis = 1 )
		error = np.sum( error ) / error.shape[0]
		sumError += error

		# print "pred image points : ", np.around(predImgP, decimals = 2)
		# print "actual image points : ", np.around(imgP.reshape(-1,2), decimals = 2)
		# print "-----------------------------"

		if vis:
			blank = np.zeros( (480, 640,3), dtype = np.uint8 )
			for pred, act in zip(predImgP, imgP.reshape(-1,2)):
				xPred, yPred = pred
				xPred = int(xPred)
				yPred = int(yPred)
				cv2.circle( blank, (xPred, yPred), 3, (0,255,0), -1 )

				xAct, yAct = act
				xAct = int( xAct )
				yAct = int( yAct )

				cv2.line( blank, (xPred, yPred), (xAct, yAct), (255,255,255), 1 )
				cv2.circle( blank, ( xAct, yAct), 3, (0,0,255), -1 )

			cv2.imshow( "test", blank )
			cv2.waitKey( 0 )
	print sumError / len( objPointList )
	## Find average error.
	return sumError / len( objPointList )

def main():
	parser = optparse.OptionParser()

	parser.add_option( '-p', type='string', action = 'store', dest = 'path',
						help = 'path to object point, image point and pantilt pos.' )

	parser.add_option( '-s', type='string', action = 'store', dest = 'savePath',
						help = 'path for save intrinsic camera matrix and distortion coeff.')

	parser.add_option( '--camMat', type='string', action = 'store', dest = 'camMatPath',
					help = 'path to camera matrix.' )

	parser.add_option( '--resultPath', type='string', action = 'store',
						dest = 'resultPath', help = 'path for saving result.' )
 
	(options, args) = parser.parse_args()

	## Load image points, object point and joint state.
	data = np.load( options.path )

	objPointList = data[ 'objectPoints' ]

	objPointList = objPointList.astype( np.float32 )
	imgPointList = data[ 'imagePoints' ]

	pantiltPosList = data[ 'pantilt' ]

	if options.camMatPath is not None:
		## Load camera matrix.
		camera_prop = np.load( options.camMatPath )
		cameraMatrix = camera_prop[ 'cameraMatrix' ]
		distCoeffs = camera_prop[ 'distCoeffs' ]
		roi = camera_prop[ 'roi' ]

	else:
		## Re-calculate camera matrix
		retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera( objPointList, imgPointList, (640,480), None, None )

		# newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix( cameraMatrix,
		# 													distCoeffs,
		# 													(640,480), 
		# 													0,
		# 													(640,480))

		distCoeffs = distCoeffs[0]

		if options.savePath is not None:
			np.savez( options.savePath,
						cameraMatrix = cameraMatrix,
						distCoeffs = distCoeffs,
						roi = roi )
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
								5.0 ] )
	# objPointList = objPointList[ : len( objPointList )/2]
	# imgPointList = imgPointList[ : len( imgPointList )/2]
	# pantiltPosList = pantiltPosList[ : len( pantiltPosList )/2]

	args = ( objPointList, imgPointList, pantiltPosList, cameraMatrix, distCoeffs )
	method = "trust-constr"
	jac = "3-point"
	hess = BFGS()
	bounds = [ (0.2,1.0), 
				(-0.1	, 0.5), 
				(0.005, 0.5), 
				(0.005, 0.5), 
				(0.035, 0.5), 
				(0.0, 0.5),
				(-0.02, 0.02 ), 
				(-90, 90) ]

	constr = LinearConstraint( np.eye(8), [i[0] for i in bounds], [i[1] for i in bounds] )

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

	lossFunction( resultTrustConstr.x, *args, vis = True )

	print "Loss Trust-constr: ", resultTrustConstr.fun
	print "Result Trust-constr: ", resultTrustConstr.x
	print cameraMatrix

	# print "Loss Evo: ", resultEvo.fun
	# print "Result Evo: ", resultEvo.x

	# finalConfig = resultEvo.x if resultEvo.fun < resultTrustConstr.fun else resultTrustConstr.x
	finalConfig = resultTrustConstr.x

	# finalConfig = list(finalConfig[:2]) + list(ORIGINAL_CONFIG[2:-1]) + list(finalConfig[2:]) 

	setNewRobotConfiguration( *finalConfig )

	if options.resultPath is not None:
		saveDimension( options.resultPath )

if __name__ == '__main__':
	main()