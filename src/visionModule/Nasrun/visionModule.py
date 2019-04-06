#!/usr/bin/env python
#
# Copyright (C) 2019  FIBO/KMUTT
#			Written by Nasrun (NeverHoliday) Hayeeyama
#


########################################################
#
#	VISION AND KINEMATIC MODULE 
#

from detect_ball_watershed import ImageProcessing
from detect_ball_svm import Kinematic


#	create instance
vision_module = ImageProcessing()
kinematic_module = Kinematic()
