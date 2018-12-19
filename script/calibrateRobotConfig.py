import numpy as np
import cv2
from scipy.optimize import minimize, LinearConstraint, BFGS, differential_evolution
import math

import optparse

from forwardkinematic import *

from utility.transformationModule import getInverseHomoMat

def lossFunction( robotConfig, objPointList, imgPointList, jsList, camMat, distCoeff ):
	setNewRobotConfiguration( *robotConfig )

	sumError = 0
	print distCoeff
	k1,k2,p1,p2,k3 = distCoeff[0] 

	for objP, imgP, js in zip( objPointList, imgPointList, jsList ):
	
		transformationMatrix = getMatrixForForwardKinematic( *js )
		invTransformationMatrix = getInverseHomoMat( transformationMatrix )
		print 'HMat', transformationMatrix

		homoObjP = np.vstack( (objP.T, np.ones( (objP.shape[0],) ) ) )
 
		predObjP_cam = np.matmul( invTransformationMatrix[:3,:], homoObjP )
		predObjP_cam /= predObjP_cam[2]

		predImgP = np.matmul( camMat, predObjP_cam )
		predImgP = predImgP[:-1]

		# print "actual", imgP[:4]
		# print "pred", predImgP.T[:4]
		# print "objP", objP[:4]
		# print predImgP.T[-1]

		error = np.linalg.norm( predImgP.T - imgP.reshape(-1,2), axis = 1 )
		error = np.sum( error ) / error.shape[0]
		sumError += error

	return sumError / len( objPointList )

def findTranslationAndRotationVector( objPointList, imgPointList, camMat, distCoeff ):

	tranVecList = []
	rotVecList = []
	for objP, imgP in zip( objPointList, imgPointList ):
		ret, rvec, tvec = cv2.solvePnP( objP, imgP, camMat, distCoeff )
		tranVecList.append( tvec )
		rotVecList.append( rvec )

	return rotVecList, tranVecList

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

	data = np.load( options.path )

	objPointList = data[ 'objectPoints' ]

	# n = len( objPointList )	

	# objPointList = np.zeros( (8*6, 3) )
	# objPointList[:,:2] = np.mgrid[:6,:8].transpose( 1,2,0 ).reshape(-1,2)[::-1]

	# objPointList = np.array( [ objPointList.copy() for i in range(n) ] )

	# objPointList[:,:,0] *= 0.0245
	# objPointList[:,:,1] *= 0.0245

	# objPointList[:,:,0] += 0.3
	# objPointList[:,:,1] -= (0.0245 * 7) / 2

	# # objPointList *= 1000

	print objPointList[2]

	objPointList = objPointList.astype( np.float32 )
	imgPointList = data[ 'imagePoints' ]
	print len(objPointList)
	print imgPointList[2]
	# sys.exit

	pantiltPosList = data[ 'pantilt' ]

	if options.camMatPath is not None:
		camera_prop = np.load( "/home/visittor/camMat.npz" )
		cameraMatrix = camera_prop[ 'cameraMatrix' ]
		distCoeffs = camera_prop[ 'distCoeffs' ]
		roi = camera_prop[ 'roi' ]

	else:

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

	initialGuess = np.array( [ 0.46,
								0.05,
								0.02,
								0.03,
								0.1,
								0.02,
								0.005,
								5.0] )
	args = ( objPointList, imgPointList, pantiltPosList, cameraMatrix, distCoeffs )
	method = "trust-constr"
	jac = "2-point"
	hess = BFGS()
	bounds = [ (0.1,1.0), (-0.1	, 0.05), (0.0, 0.05), (0.0, 0.05), (0.0, 0.05), (0.0, 0.05),
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