#!/usr/bin/env python

import numpy as np
from visionManager.visionModule import VisionModule, KinematicModule

class CameraKinematic( object ):
    """
    Kinematics from base to camera
    argument :  
        x : (float) distance of x
        y : (float) distance of y
        offsetFrameBaseToPan : (float) distance from base frame to frame 1 ( pan frame )
        offsetFramePantoTilt : (float) distance from pan frame to frame 2 ( tilt frame )

    NOTE :
        I get rotation matrix and translation vector from matlab by using forward kinematics script.
        For understanding, check how to calculate forward kinematic or attend intro to robotics class
    
        But in the future we will have homograpy matrix to find automate rotation matrix
        ... not the future
    """ 

    def __init__( self, x, y, offsetFrameBaseToPan, offsetFramePantoTilt ):
        
        self.distance = np.sqrt( np.power( x, 2 ) + np.power( y, 2 ) )
        self.angle = np.arctan( float( y ) / float( x ) )
        self.offsetFrameBaseToPan = offsetFrameBaseToPan
        self.offsetBaseToTilt = offsetFramePantoTilt

    def updateMatrix( self, qPan, qTilt ):
        """
        update q (joint state) in homogenous transformation matrix
        argument : 
            qPan : (float) joint state at pan joint (q1)
            qTilt : (float) joint state at tilt joint (q2)
        """

        #   Shorten func cos and sin
        cos = np.cos
        sin = np.sin

        qTilt *= -1

        #   Rotation matrix of camera
        rotationMatrix =  [ [  cos( self.angle + qTilt )*cos( qPan ), -sin( self.angle + qTilt )*cos( qPan ),  sin( qPan )  ],
                            [  cos( self.angle + qTilt )*sin( qPan ), -sin( self.angle + qTilt )*sin( qPan ), -cos( qPan )  ],
                            [              sin( self.angle + qTilt ),              cos( self.angle + qTilt ),            0  ] ]

        translationVec = np.array( [ 0.0, 0.0, 0.0 ], np.float64 )
        rotationVec = np.array( [ 0.0, np.pi/2, np.pi ], np.float64 )
        #   multiply
        rotation = KinematicModule.create_transformationMatrix( translationVec, rotationVec, 'zyz' )

        #   Translation vector of camera
        translationVector = [ self.distance * cos( self.angle + qTilt ) * cos( qPan ), 
                              self.distance * cos( self.angle + qTilt) * sin( qPan ),
                              self.offsetFrameBaseToPan + self.offsetBaseToTilt + self.distance * sin( self.angle + qTilt ) ]
        #   Convert to numpy
        rotationMatrixNumpy = np.array( rotationMatrix )
        translationVectorNumpy = np.array( translationVector )

        #   Push rotation matrix and translation vector to homogenous transformation matrix
        homogenousTransformationMatrix = np.eye( 4 )
        homogenousTransformationMatrix[ 0:3, 0:3 ] = rotationMatrixNumpy
        homogenousTransformationMatrix[ 0:3, 3 ] = translationVectorNumpy
        homogenousTransformationMatrix = np.matmul( homogenousTransformationMatrix, rotation )


        return homogenousTransformationMatrix

if __name__ == "__main__":
    
    camera = CameraKinematic( 2, 3, 10, 4 )

    print camera.updateMatrix( 1, 2 )   
    