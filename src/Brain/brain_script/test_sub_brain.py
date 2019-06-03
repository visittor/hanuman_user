#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun Hayeeyama
#

########################################################
#
#	STANDARD IMPORTS
#

import sys
import os

########################################################
#
#	LOCAL IMPORTS
#

from brain.brainState import FSMBrainState
from brain.HanumanRosInterface import HanumanRosInterface

from newbie_hanuman.msg import postDictMsg

import numpy as np
import math

import time
import rospy

########################################################
#
#	GLOBALS
#

########################################################
#
#	EXCEPTION DEFINITIONS
#

########################################################
#
#	HELPER FUNCTIONS
#

########################################################
#
#	CLASS DEFINITIONS
#
class Test1( FSMBrainState ):
    def __init__( self, nextState = "None" ):
        super( Test1, self ).__init__( "Test1" )

        self.previousTime = None

        self.nextState = nextState

        self.addSubBrain( Sub1Test1( nextState="Sub2Test1" ) )
        self.addSubBrain( Sub2Test1( nextState="Sub1Test1" ) )

        self.setFirstSubBrain( "Sub1Test1" )

    def firstStep( self ):
        
        rospy.logdebug( "Enter {} state".format( self.name ) )

        self.previousTime = time.time()

    def step( self ):

        currentTime = time.time() - self.previousTime

        rospy.loginfo( "    At {}, Time : {}".format( self.name, currentTime ) )

        if currentTime >= 5.0:

            self.SignalChangeSubBrain( self.nextState )

class Sub1Test1( FSMBrainState ):

    def __init__( self, nextState = "None" ):
        super( Sub1Test1, self ).__init__( "Sub1Test1" )

        self.previousTime = None

        self.nextState = nextState

    def firstStep( self ):
        
        rospy.logdebug( "Enter {} state".format( self.name ) )

        self.previousTime = time.time()

    def step( self ):

        currentTime = time.time() - self.previousTime

        rospy.loginfo( "    At {}, Time : {}".format( self.name, currentTime ) )

        if currentTime >= 10.0:

            self.SignalChangeSubBrain( self.nextState )

class Sub2Test1( FSMBrainState ):

    def __init__( self, nextState = "None" ):
        super( Sub2Test1, self ).__init__( "Sub2Test1" )

        self.previousTime = None

        self.nextState = nextState

    def firstStep( self ):
        
        rospy.logdebug( "Enter {} state".format( self.name ) )

        self.previousTime = time.time()

    def step( self ):

        currentTime = time.time() - self.previousTime

        rospy.loginfo( "    At {}, Time : {}".format( self.name, currentTime ) )

        if currentTime >= 10.0:

            self.SignalChangeSubBrain( self.nextState )


class Test2( FSMBrainState ):

    def __init__( self, nextState = "None" ):
        super( Test2, self ).__init__( "Test2" )

        self.previousTime = None
        self.nextState = nextState

    def firstStep( self ):

        rospy.logdebug( "Enter {} state".format( self.name ) )

        self.previousTime = time.time()

    def step( self ):

        currentTime = time.time() - self.previousTime

        rospy.loginfo( "    At {}, Time : {}".format( self.name, currentTime ) )

        if currentTime >= 3.0:

            self.SignalChangeSubBrain( self.nextState )

class AllBrain( FSMBrainState ):

    def __init__( self ):
        super( AllBrain, self ).__init__( "AllBrain" )

        self.addSubBrain( Test1( nextState="Test2" ) )
        self.addSubBrain( Test2( nextState="Test1" ) )

        self.setFirstSubBrain( "Test1" )



main_brain = AllBrain()

