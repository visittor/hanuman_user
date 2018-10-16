#!/usr/bin/env python

from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

import numpy as np

import time
import rospy
 
class TrackingBall( FSMBrainState ):
    
    def __init__( self ):
        super( TrackingBall, self ).__init__( 'Tracking Ball' )
        
        
    def firstStep( self ):
        '''
            first step before execute this brain
        '''
        self.initTime = time.time()

    def step( self ):
        
        #   if self.rosInterface.visionManager
        #   Is ball detect ?
        #   if detect
        #       send service to track that ball
        #   else
        #       terminate
        pass

