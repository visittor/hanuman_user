import numpy as np
import cv2
from util import *
from region_reciever import Region_reciver

class Robot_provider(Region_reciver):

	def __init__(self, angle_threshold = None, size_ratio=1):
		super( Robot_provider, self).__init__()
		super( Robot_provider, self).set_size_ratio(size_ratio)
		super( Robot_provider, self).set_angle_threshold(angle_threshold)
		self._region = None
		self._united_region = []
		self.__robot = {"Opponent":[], "Ally":[], "Unknown":[]}

	def receive_region(self, region, opponent_color=0, ally_color=1):
		self._region = region
		ally_region = super(Robot_provider, self).unite_region(self._region, ally_color)
		opponent_region = super(Robot_provider, self).unite_region(self._region, opponent_color)
		ally_list = super(Robot_provider, self).link_list_to_list(ally_region)
		opponent_list = super(Robot_provider, self).link_list_to_list(opponent_region)
		self.__robot["Opponent"] = [ RobotLandmark(max(r[0],key=lambda x:x[0]),"Opponent") for r in opponent_list]
		self.__robot["Ally"] = [ RobotLandmark(max(r[0],key=lambda x:x[0]),"Ally") for r in ally_list]
		self.__robot["Unknown"] = []
		self.__robot["Opponent"] = self.__do_filter(self.__robot["Opponent"])
		self.__robot["Ally"] = self.__do_filter(self.__robot["Ally"])

	def __do_filter(self, robot_list):
		filtered_ = []
		while len(robot_list) > 0:
			robot = robot_list.pop(0)
			ii = 0
			while ii < len(robot_list):
				if -100 < robot.footPos[1] - robot_list[ii].footPos[1] < 100:
					robot_list.pop(ii)
				else:
					ii += 1
			filtered_.append(robot)
		return filtered_

	def get_robotLandmark(self):
		return self.__robot

class RobotLandmark(object):

	def __init__(self, foot, role = "Unknown"):
		self.__foot = foot
		self.__role = role

	@property
	def footPos(self):
		return self.__foot

	@property
	def role(self):
		return self.__role