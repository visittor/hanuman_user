#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import optparse


class Camera( object ):
    '''
        open camera object
    '''
    def __init__( self, devInfo ):

        #   init publisher 
        self.publisher = rospy.Publisher( '/camera/image_raw', Image, queue_size = 1 )

        #   init node
        rospy.init_node( 'camera' )

        #   init rate
        rospy.Rate( 30 )

        #   select id
        if devInfo == '/dev/video0':    
            cameraId = 0
        elif devInfo == '/dev/video1':
            cameraId = 1
        else:
            cameraId = 0

        #   init camera
        self.camera = cv2.VideoCapture( cameraId )

        #   init cv bridge
        self.bridge = CvBridge()

        #   other parameter
        self.status = False
        self.frame = None
        self.width = None
        self.height = None

    def openCamera( self ):
        '''
            open camera
        '''
        self.status, self.frame = self.camera.read()
        self.width, self.height = self.frame.shape[ 1 ], self.frame.shape[ 0 ]

    def publish( self ):
        '''
            publish method
        '''
        #   change to image message
        imgMessage = self.bridge.cv2_to_imgmsg( self.frame, "bgr8" )

        try:
            #   publish !
            self.publisher.publish( imgMessage )

        except CvBridgeError as e:
            print e

if __name__ == '__main__':
    
    parser = optparse.OptionParser()

    parser.add_option( '--deviceCamera', dest='deviceCamera', type = 'str', action='store',
                        default='/dev/video0', help='input your camera device port' )

    ( options, args ) = parser.parse_args()

    #   get options
    cameraDevice = options.deviceCamera
    camera = Camera( cameraDevice )
    
    rospy.loginfo( "Open camera with {} ".format( cameraDevice ) )

    try:
        while not rospy.is_shutdown():
            camera.openCamera()
            camera.publish()

    except rospy.ROSInterruptException:
        pass