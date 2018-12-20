import numpy as np
from numpy import sin as s
from numpy import cos as c

import configobj

import math
## Robot dimension in m
# _H = 0.46
# _L = 0.05
# _h1 = 0.02
# _l1 = 0.03
# _h2 = 0.1
# _l2 = 0.02
# _lx = 0.005
# _Phi = math.radians( 5.0 )
_H = 4.41522754e-01
_L = 8.78644672e-03
_h1 = 1.23728668e-02
_l1 = 1.87019168e-03
_h2 = 0.0
_l2 = 5.00000000e-02
_lx = 9.44950202e-03
_Phi = math.radians( 1.43430832e+01 )

_prefix = 1e-0

H = _H
L = _L
h1 = _h1
l1 = _l1
h2 = _h2
l2 = _l2
lx = _lx
Phi = _Phi

def getMatrixForForwardKinematic( *q ):
	'''
	Forward kinematic from robot base to camera.
	DH table
	=================================================
	| index	|	theta	|	d 	|	r 	| 	alpha	|
	=================================================
	|	1	|	0		|	H	|	L	|	-pi/2	|
	-------------------------------------------------
	|	2	|	phi		|	0	|	0	|	 pi/2	|
	=================================================
	DH table from robot base to pan-tilt base.^
	-------------------------------------------------
	DH table from pan-tilt base to camera. v
	=================================================
	|	3	|	q1		|	h1	|	l1	|	-pi/2	|
	-------------------------------------------------
	|	4	|	q2-pi/2	|	0	|	h2	|	-pi/2	|
	-------------------------------------------------
	|	5	|	pi/2	|	l2	|	lx	|	0		|
	=================================================
	'''
	global H, L, h1, l1, h2, l2, Phi, lx
	q1 = q[0]
	q2 = q[1]
	q3 = q2 - ( np.pi / 2 )

	H_ = _prefix * _H
	L_ = _prefix * _L
	h1_ = _prefix * _h1
	l1_ = _prefix * _l1
	h2_ = _prefix * _h2
	l2_ = _prefix * _l2
	lx_ = _prefix * _lx
	
	T_R_j3 = [
				[  c(Phi)*s(q1), 	-c(q2)*s(Phi) - c(Phi)*c(q1)*s(q2),		c(Phi)*c(q1)*c(q2) - s(Phi)*s(q2), 		L_ + h1_*s(Phi) + l1_*c(Phi)*c(q1) + h2_*c(q2)*s(Phi) - l2_*s(Phi)*s(q2) + l2_*c(Phi)*c(q1)*c(q2) + h2_*c(Phi)*c(q1)*s(q2) - c(Phi)*s(q1)*lx_],
				[  -c(q1),          -s(q1)*s(q2),                     		c(q2)*s(q1),                        	s(q1)*(l1_ + l2_*c(q2) + h2_*s(q2))],
				[  -s(Phi)*s(q1),   c(q1)*s(Phi)*s(q2) - c(Phi)*c(q2), 		-c(Phi)*s(q2) - c(q1)*c(q2)*s(Phi), 	H_ + h1_*c(Phi) + h2_*c(Phi)*c(q2) - l1_*c(q1)*s(Phi) - l2_*c(Phi)*s(q2) - l2_*c(q1)*c(q2)*s(Phi) - h2_*c(q1)*s(Phi)*s(q2)],
				[  0,               0,                                  	0,                                  	1]
			 ]
	T_R_j3 = np.array( T_R_j3, dtype = np.float64 )

	return T_R_j3



def setPrefix( n ):
	'''
	Set prefix for length. Currently this module use meters.
	argument:
		n 	:	( float ) prefix.
	'''
	global _prefix

	_prefix = n

def setNewRobotConfiguration( HNew, LNew, h1New, l1New, h2New, l2New, lxNew, phiNew ):
	'''
	Set robot configuration.
	H, L, h1, l1, h2, l2, lx and phi respectively.
	Here is DH-table for forwardkinematic.

	Forward kinematic from robot base to camera.
	DH table
	=================================================
	| index	|	theta	|	d 	|	r 	| 	alpha	|
	=================================================
	|	1	|	0		|	H	|	L	|	-pi/2	|
	-------------------------------------------------
	|	2	|	phi		|	0	|	0	|	 pi/2	|
	=================================================
	DH table from robot base to pan-tilt base.^
	-------------------------------------------------
	DH table from pan-tilt base to camera. v
	=================================================
	|	3	|	q1		|	h1	|	l1	|	-pi/2	|
	-------------------------------------------------
	|	4	|	q2-pi/2	|	0	|	h2	|	-pi/2	|
	-------------------------------------------------
	|	5	|	pi/2	|	l2	|	lx	|	0		|
	=================================================
	
	arguments:
		H, L, h1, l1, h2, l2, lx and phi respectively. ( float )

	'''
	global H, L, h1, l1, h2, l2, Phi, lx

	H = HNew
	L = LNew
	h1 = h1New
	l1 = l1New
	h2 = h2New
	l2 = l2New
	lx = lxNew
	Phi = math.radians( phiNew )

def resetConfiguration( ):
	global H, L, h1, l1, h2, l2, Phi, lx

	H = _H
	L = _L
	h1 = _h1
	l1 = _l1
	h2 = _h2
	l2 = _l2
	lx = _lx
	Phi = _Phi

def loadDimensionFromConfig( filePath ):
	'''
	Load robot dimension from config file.
	argument:
		filePath	:	( str )
	'''
	config = configobj.ConfigObj( filePath )[ 'RobotDimension' ]

	global H, L, h1, l1, h2, l2, Phi, lx

	H = float( config['H'] )
	L = float( config['L'] )
	h1 = float( config['h1'] )
	l1 = float( config['l1'] )
	h2 = float( config['h2'] )
	l2 = float( config['l2'] )
	lx = float( config['lx'] )
	Phi = math.radians( float( config['Phi'] ) )

def saveDimension( filePath ):
	'''
	Save current robot dimension to config file.
	arguments:
		filePath	:	( str )
	'''
	dimensionDict = { 	'H' : 	H,
						'L'	:	L,
						'h1':	h1,
						'l1':	l1,
						'h2':	h2,
						'l2':	l2,
						'lx':	lx,
						'Phi':	math.degrees( Phi )
					}

	config = configobj.ConfigObj()
	config[ 'RobotDimension' ] = dimensionDict

	config.filename = filePath

	config.write()