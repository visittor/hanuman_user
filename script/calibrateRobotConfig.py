import numpy as np
import cv2
from scipy.optimize import minimize, LinearConstraint, BFGS, differential_evolution
import math

import optparse

from forwardkinematic import *

from utility.transformationModule import getInverseHomoMat

def lossFunction( robotConfig, objPointList, imgPointList, jsList, camMat, distCoeff ):
	'''
	Loss function for minimizer. Project point in 3d coor to image plane then find
	and distance between projected point with actual image point.
	'''
	## Set robot dimension.
	setNewRobotConfiguration( *robotConfig )

	## Error
	sumError = 0

	## Loop for every points.
	for objP, imgP, js in zip( objPointList, imgPointList, jsList ):
	
		## Get transformation matrix .
		transformationMatrix = getMatrixForForwardKinematic( *js )
		invTransformationMatrix = getInverseHomoMat( transformationMatrix )
		
		## Print this. To indicate that program is still running.
		print 'HMat', transformationMatrix

		## Get homogenouse point
		homoObjP = np.vstack( (objP.T, np.ones( (objP.shape[0],) ) ) )
 
 		## Project point to image plane.
		predObjP_cam = np.matmul( invTransformationMatrix[:3,:], homoObjP )
		predObjP_cam /= predObjP_cam[2]

		predImgP = np.matmul( camMat, predObjP_cam )
		predImgP = predImgP[:-1]

		## Find error for each points.
		error = np.linalg.norm( predImgP.T - imgP.reshape(-1,2), axis = 1 )
		error = np.sum( error ) / error.shape[0]
		sumError += error

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
		camera_prop = np.load( "/home/visittor/camMat.npz" )
		cameraMatrix = camera_prop[ 'cameraMatrix' ]
		distCoeffs = camera_prop[ 'distCoeffs' ]
		roi = camera_prop[ 'roi' ]

	else:
		## Re-calculate camera matrix
		retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera( objPointList, imgPointList, (640,480), None, None )

		if options.savePath is not None:
			newcameramtx, roi = cv2.getOptimalNewCameraMatrix( cameraMatrix,
															distCoeffs,
															(640,480), 
															0,
															(640,480))
			np.savez( options.savePath,
						cameraMatrix = newcameramtx,
						distCoeffs = distCoeffs,
						roi = roi )
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

	args = ( objPointList, imgPointList, pantiltPosList, cameraMatrix, distCoeffs )
	method = "trust-constr"
	jac = "2-point"
	hess = BFGS()
	bounds = [ (0.1,1.0), (-0.1	, 0.05), (0.005, 0.05), (0.005, 0.05), (0.035, 0.1), (0.0, 0.05),
			(-0.02, 0.02 ), (-90, 90) ]

	constr = LinearConstraint( np.eye(8), [i[0] for i in bounds], [i[1] for i in bounds] )

	resultTrustConstr = minimize( lossFunction, initialGuess,
						args = args,
						method = method,
						jac = jac,
						hess = hess,
						bounds = bounds,
						constraints = constr )

	resultEvo = differential_evolution(lossFunction, bounds, 
									args = args, 
									)

	print "Loss Trust-constr: ", resultTrustConstr.fun
	print "Result Trust-constr: ", resultTrustConstr.x

	print "Loss Evo: ", resultEvo.fun
	print "Result Evo: ", resultEvo.x

	finalConfig = resultEvo.x if resultEvo.fun < resultTrustConstr.fun else resultTrustConstr.x

	setNewRobotConfiguration( *finalConfig )

	if options.resultPath is not None:
		saveDimension( options.resultPath )

if __name__ == '__main__':
	main()