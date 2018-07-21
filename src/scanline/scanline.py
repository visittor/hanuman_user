import scanlineLib
import numpy as np
import cv2
from util import *

class ScanLine(object):
	
	def __init__(self, **kwarg):
		# self._region_output = np.zeros((1,4))
		# self.color_list = np.array([ [[0,0,0] ,[255,255,255]],
		# 					], dtype = np.uint8)
		# self._scan_output = np.zeros((1,3))
		# self.scan_axis = 1
		# self.grid_dis = 25
		# self.step = 1
		# self.co = 10
		if "color_list" in kwarg:
			self.set_color_list(kwarg["color_list"])
		else:
			self.set_color_list(np.array([ [[0,0,0] ,[255,255,255]],], dtype = np.uint8))

		self.scan_axis = kwarg.get('scan_axis', 1)
		self.grid_dis = kwarg.get('grid_dis', 25)
		self.step = kwarg.get('step', 1)
		self.co = kwarg.get('co', 10)

	def set_color_list(self, colorList):
		if colorList.ndim != 3:
			raise ValueError("color_list's ndim != 3")
		elif colorList.shape[1] != 2 or colorList.shape[2] != 3:
			raise ValueError("color_list's shape not equal (n ,2 ,3)")
		self.color_list = colorList

	def scan_image(self, img_hsv, horizon):
		if self.scan_axis == 1:
			self._scan_output = scanlineLib.scan2DVerticle(img_hsv, self.color_list, self.grid_dis, self.co, horizon)
		else:
			self._scan_output =  scanlineLib.scan2DHorizon(img_hsv, self.color_list, self.grid_dis, horizon)

	def visualize_scan_line(self, img, color_obj):
		for scanline in self._scan_output:
			for p in scanline:
				for color in color_obj.values():
					if p[2] == color.index:
						img[p[1],p[0]] = color.RenderColor_RGB

	def find_region(self, img_hsv, horizon = 0, minPix = 2):
		self.scan_image(img_hsv, horizon)
		self._region_output = [ scanlineLib.scanline2region(i, minPix) for i in self._scan_output]
		# self._region_output = scanlineLib.scanlines2regions(self._scan_output, minPix = minPix)
		
	def visualize_region(self, img, color_obj):
		for regions in self._region_output:
			for region in regions:
				for color in color_obj:
					if region[4] == color.index:
						color = tuple(color.RenderColor_RGB)
						cv2.line(img, tuple(region[:2]), tuple(region[2:4]), color, self.grid_dis/5, 4)

	def clip_region(self, color_index):
		ii = 0
		boundary = []
		for i,region in enumerate(self._region_output):
			j = region.shape[0] - 1
			index_list = range(0, j+1)
			while j >= 0:
				if region[j,4] == color_index:
					boundary.append(region[j,2:4])
					index_list = range(j+1, region.shape[0])
					break
				j -= 1
			self._region_output[i] = np.delete(region, index_list, axis = 0)
		return np.array(boundary)

	def get_regions(self):
		return [[Region(r) for r in rs] for rs in self._region_output]

	@property
	def info(self):
		a = 'scan_axis = {}\ngrid_dis = {}\nstep = {}\nco = {}\ncolor_list = {}'.format(self.scan_axis, self.grid_dis, self.step, self.co, self.color_list)
		return a

	def __len__(self):
		return len(self._region_output)

class scanline_polygon(object):
	def __init__(self, **kwarg):
		if "color_list" in kwarg:
			self.set_color_list(kwarg["color_list"])
		else:
			self.set_color_list(np.array([ [[0,0,0] ,[255,255,255]],], dtype = np.uint8))

		self.step = kwarg.get('step', 1)
		self.minPix = kwarg.get('minPix', 10)

	def set_color_list(self, colorList):
		if colorList.ndim != 3:
			raise ValueError("color_list's ndim != 3")
		elif colorList.shape[1] != 2 or colorList.shape[2] != 3:
			raise ValueError("color_list's shape not equal (n ,2 ,3)")
		self.color_list = colorList

	def scan_image(self, img_hsv, boundary):
		self._scan_output = scanlineLib.scanPolygon(img_hsv, self.color_list, boundary, step = self.step)

	def find_region(self, img_hsv, boundary, end_scan = -1):
		self.scan_image(img_hsv, boundary)
		self._region_output = scanlineLib.scanlines2regions([self._scan_output], minPix = self.minPix)

	def visualize(self,img,color_dict):
		for regions in self._region_output:
			for r in regions:
				for color in color_dict.values():
					if r[4] == color.index:
						cv2.line(img, (r[0], r[1]), (r[2], r[3]), color.RenderColor_RGB, 2)

	def get_regions(self):
		return [[Region(r) for r in rs] for rs in self._region_output]
		
	def __getitem__(self, index):
		return self.region_for_polygon(self._region_output[index])

	@property
	def lenght(self):
		return len(self._region_output)

class Region(object):
	def __init__(self, region):
		self.region = region

	@property
	def start(self):
		return self.region[:2]

	@property
	def stop(self):
		return self.region[2:4]

	@property
	def middle(self):
		return (self.region[:2] + self.region[2:4])/2

	@property
	def color(self):
		return self.region[4]

	@property
	def length(self):
		return np.linalg.norm(self.region[:2] - self.region[2:4])

	def __sub__(self, b):
		a = self.middle - b.middle
		return a