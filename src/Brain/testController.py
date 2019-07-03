#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by EIEI
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

from brain_script.gameControl_fsmbrainstate import Controller
from brain_script.gameControl_brainstate import InitialState, SetState, PlayState, FinishState, PernaltyState
# from brain_script.gameControl_brainstate import ReadyState
from brain_script.abnormal_brainstate import AbnormalState

from enter_field.EnterField import ReadyState


from gotoandkick_fsmbrainstate import _IDLE, MainBrain

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

initialState = InitialState()
readyState = ReadyState()
setState = SetState()
playState = MainBrain()
finishState = _IDLE()
penaltyState = PernaltyState()

abnormalState = AbnormalState()

main_brain = Controller(
    initialState = initialState,
    readyState = readyState,
    setState = setState,
    playState = playState,
    finishState = finishState,
    penaltyState = penaltyState,
    abnormalState = abnormalState
)