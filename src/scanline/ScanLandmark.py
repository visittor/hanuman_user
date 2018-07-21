import numpy as np
import cv2
from scanline import ScanLine,scanline_polygon
from line_provider import Line_provider, VerticleLine_provider
from Goal_provider import Goal_provider
from cross_provider import Cross_provider
from Robot_provider import Robot_provider
import time

class ScanLandmark(object):
	class Color(object):
		def __init__(self, id_, name, RenderColor_RGB, color_min, color_max):
			self.id = id_
			self.index = id_ - 1
			self.name = name
			self.RenderColor_RGB = RenderColor_RGB
			self.color_min = color_min
			self.color_max = color_max

	def __init__(self, config, **kwarg):
		'''
		config is a configobj object from color_config.
		if is_do_horizontal is False, get_goals always return None.
		'''
		line_angleThreshold = kwarg.get("line_angleThreshold", 0.4)
		line_sizeRatio = kwarg.get("line_sizeRatio", 0.5)
		goal_angleThreshold = kwarg.get("goal_angleThreshold", 0.4)
		goal_sizeRatio = kwarg.get("goal_sizeRatio", 0.1)
		verticalGridDis = kwarg.get("verticalGridDis", 20)
		horizontalGridis = kwarg.get("horizontalGridis", 15)
		verticalCO = kwarg.get("verticalCO",1000)
		horizontalstep = kwarg.get("horizontalstep",2)
		verticalstep = kwarg.get("verticalstep", 1)
		polygonMinPix = kwarg.get('polygonMinPix', 8)

		self.frechet_d_thr = kwarg.get("frechet_d_thr", 15)
		self.is_do_horizontal = kwarg.get("is_do_horizontal", True)
		self.img_w = kwarg.get("img_w", 640)
		self.img_h = kwarg.get("img_h", 480)
		self.cross_check_tolerance = kwarg.get("cross_check_tolerance", 40)
		self.minPix = kwarg.get('minPix', 1)
		self.polygon_scan_step = kwarg.get("polygon_scan_step", 1)
		self.config = config
		self.color_list = None
		self.color_dict = {}
		self.__create_color_list()

		self.vertical_scan = ScanLine(color_list = self.color_list, 
							grid_dis = verticalGridDis,
							scan_axis = 1,
							co = verticalCO,
							step = verticalstep)
		self.horizontal_scan = ScanLine(color_list = self.color_list,
										grid_dis = horizontalGridis,
										scan_axis = 0,
										co = 1,
										step = horizontalstep)
		self.polygon_scanline = scanline_polygon(color_list = self.color_list, step = self.polygon_scan_step, minPix = polygonMinPix)
		self.line_p = Line_provider(angle_threshold=line_angleThreshold, size_ratio=line_sizeRatio)
		self.verLine_p = VerticleLine_provider()
		self.cross_p = Cross_provider()
		self.goal_p = Goal_provider(goal_angleThreshold, goal_sizeRatio, h2w_ratio= 2.0, distance_thr= 75)
		self.robot_p = Robot_provider()
		self.img_hsv = None

	def __create_color_list(self):
		colorDefinition = self.config["ColorDefinitions"]
		self.color_list =np.zeros((len(colorDefinition),2,3),dtype = np.uint8)
		for i,(_,color) in enumerate(colorDefinition.items()):
			id_ = eval(color["ID"])
			name = color["Name"]
			RenderColor_RGB = eval(color["RenderColor_RGB"])
			H_Max = eval(color["H_Max"])
			H_Min = eval(color["H_Min"])
			S_Max = eval(color["S_Max"])
			S_Min = eval(color["S_Min"])
			V_Max = eval(color["V_Max"])
			V_Min = eval(color["V_Min"])
			self.color_list[i,0] = np.array([H_Min,S_Min,V_Min], dtype = np.uint8)
			self.color_list[i,1] = np.array([H_Max,S_Max,V_Max], dtype = np.uint8)
			self.color_dict[name] = self.Color(id_, name, RenderColor_RGB, np.array([H_Min,S_Min,V_Min], dtype=np.uint8), np.array([H_Max,S_Max,V_Max], dtype=np.uint8))

	def do_scan_image(self, img, cvt2hsv=True, horizon = 0, end_scan = -1):
		'''Must be called first. This function scan image with ScanLine. Do vertical scan first
		then horizontal scan if is_do_horizontal is True. This function return a boundary of green 
		area in an image.
		img is image to be scaned.
		cvt2hsv if True, will convert img from rgb to hsv first.
		horizon can be calculated by tilt angle and/or robot kinematic and/or infomation from IMU
		vertical ScanLine will start scan at y = horizon and end at y = end_scan if end_scan = -1 it will end
		at bottom of image.
		'''
		img_hsv = img.copy() if not cvt2hsv else cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		self.img_hsv = img_hsv.copy()
		self.vertical_scan.find_region(img_hsv, horizon= horizon, minPix = self.minPix)
		boundary = self.vertical_scan.clip_region(self.color_dict["green"].index)
		if len(boundary) > 0:
			horizon_end_scan = max(boundary, key = lambda x : x[1])[1]
			boundary = np.vstack([boundary, np.array([[img_hsv.shape[1]-1, img_hsv.shape[0]-1]])])
			boundary = np.vstack([boundary, np.array([[0,img_hsv.shape[0]-1]])])
		else:
			horizon_end_scan = img_hsv.shape[0]
			boundary = np.array([[img_hsv.shape[1]-1, img_hsv.shape[0]-1], [0,img_hsv.shape[0]-1]])

		convexhull = cv2.convexHull(np.array([boundary]))

		boundary_ = convexhull[ np.where(convexhull[:,0,1] < img_hsv.shape[0] - 2) ].reshape(-1,2).astype(np.int)

		self.polygon_scanline.find_region(img_hsv, boundary_ + 5)

		if self.is_do_horizontal:
			cv2.drawContours(img_hsv, [cv2.convexHull(np.array([boundary]))], 0, (0,0,0), -1)
			self.horizontal_scan.find_region(img_hsv, horizon= 0, minPix = self.minPix)

		return boundary

	def get_vertical_regions(self):
		'''
		Must be called after do_scan_image. Return list of instance of Region class
		for vertical scanline(See definition at scanline.py).
		'''
		return self.vertical_scan.get_regions()

	def get_horizontal_regions(self):
		'''
		Must be called after do_scan_image. Return list of instance of region class
		for vertical scanline(See definition at scanline.py). If pass is_do_horizon = False 
		to ScanLandmark constructor, this function always return None. 
		'''
		return self.horizontal_scan.get_regions() if self.is_do_horizontal else None

	def get_lines(self):
		'''Must be called after do_scan_image. This function find a line field in an image.
		This function return list of instance of Line class(See definition at line_provider.py).
		'''
		self.line_p.receive_region(self.vertical_scan.get_regions(), connected_color=self.color_dict["white"].index)
		return self.line_p.get_lines(axis= 1, frechet_distance= self.frechet_d_thr)

	def get_verLine(self):
		return self.verLine_p.get_verticalLine()

	def get_crosses(self):
		''' Must be called after get_lines. This function find a cross in an image.
		This function return list of instance of Cross class(See definition at cross_provider.py).
		'''
		self.cross_p.receive_line(self.line_p, max_h=self.img_h, max_w=self.img_w, tolerance= self.cross_check_tolerance)
		return self.cross_p.get_point()

	def get_goals(self):
		''' Must be called after do_scan_image. If pass is_do_horizon = False to ScanLandmark constructor, 
		this function always return None.Otherwise, return two point for goal position. 
		'''
		# if self.is_do_horizontal:
		# 	self.goal_p.receive_region(self.horizontal_scan, connected_color=self.color_dict["white"].index)
		# 	return self.goal_p.get_filtred_Squar(boundary= cv2.convexHull(np.array([boundary])))
		self.goal_p.receive_region(self.img_hsv, self.polygon_scanline.get_regions()[0], self.verLine_p.get_verticalLine(), self.color_dict)
		goal = self.goal_p.get_goal()
		return goal

	def get_robots(self, opponent_color, ally_color):
		self.robot_p.receive_region(self.horizontal_scan, self.color_dict[opponent_color.lower()].index,self.color_dict[ally_color.lower()].index)
		return self.robot_p.get_robotLandmark()

	def visualize_vertical_region(self, img):
		self.vertical_scan.visualize_region(img, self.color_dict.values())

	def visualize_horizontal_region(self, img):
		self.horizontal_scan.visualize_region(img, self.color_dict.values())

	def visualize_node(self, img):
		self.line_p.visualize_node(img)

	def visualize_polygon_scanline(self, img):
		self.polygon_scanline.visualize(img, self.color_dict)