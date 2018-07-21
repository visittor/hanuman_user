#! /usr/bin/env python
import numpy as np 
import cv2

from scanline import ScanLandmark

from visionManager.visionModule import VisionModule 
from utility.performanceX import StopWatch

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty

import sys
PATH = sys.path[0]

class ImageProcessing(VisionModule):
	HEIGHT = 0.6
	DISTANCE = np.inf
	def __init__(self):
		super(ImageProcessing, self).__init__()
		## TODO : edit line below
		self.objectsMsgType = Empty
		self.subscribePantiltTopic = True

		path = "/".join(PATH.split("/")[:]+["colordef.ini"])
		config = configobj.ConfigObj(path)

		path = "/".join(PATH.split("/")[:]+["output_matrix.npz"])
		intrinMat = np.load(path)["camera_matrix"]
		self.__fy = intrinMat[1,1]
		self.__cy = intrinMat[1,2]

		##argument for class
		kwarg = {'line_angleThreshold':0.3, 'is_do_horizontal':False,
		 'minPix': 1}

		 self.scanlendmark = ScanLandmark(config, kwarg)

	def __calculateHorizon(self, js):
		## TODO : complete this function.
		tilt = js.position[js.name.index('tilt')]
		horizon = self.__fy*((self.HEIGHT*np.cos(tilt) - \
							self.DISTANCE*np.sin(tilt))/ \
							(self.HEIGHT*np.sin(tilt) + \
							self.DISTANCE*np.cos(tilt))) + self.__cy 
		return 0

	def __regionsToPose2D(self, regions):
		return [Pose2D(x=r.middle[0], y=r.middle[1],z=0,w=0) for r in regions]

	def ImageProcessingFunction(self, img ,header, js):
		img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		## scanline stuff.
		horizon = self.__calculateHorizon(js)
		boundary = self.scanlendmark(img_hsv, cvt2hsv=False, horizon=horizon)
		verticalRegion = self.scanlendmark.get_vertical_regions()
		whiteStips = self.__regionsToPose2D(verticalRegion)

		## TODO : write a ball finding process below.

		return Empty()

	def visualizeFunction(self, img, msg):
		self.scanlendmark.visualize_vertical_region(img)