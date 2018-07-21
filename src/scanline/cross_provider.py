from line_provider import Line_provider
from util import *
import numpy as np 
import cv2


class Cross_provider(object):

	def __init__(self):
		self._lines = None
		self._points = []
		self.crossing = []
		self._path = []
		self._smt = []

	def receive_line(self, line_, max_h = 480, max_w = 640, tolerance = 40):
		if line_.__class__ == Line_provider:
			self._lines = line_
			self._points = []
			self.crossing = []
			self.find_all_cross(max_h, max_w)
			self.cross_check(self._lines.crossing, tolerance)
		else:
			raise ValueError("line_ is not class Line_provider.Line")

	def initial_(self):
		for i,l in enumerate(self._lines):
			self._points.append(l.start)
			self._points.append(l.stop)
			self._smt.append([2*i,2*i+1])

	def find_intersection_point(self, l1, l2):
		point = find_line_intersection(l1.m, l1.c, l2.m, l2.c)
		return point

	def find_all_cross(self, max_h, max_w):
		ii = 0
		while ii < self._lines.lenght:
			jj = ii + 1
			while jj < self._lines.lenght:
				point_ = self.find_intersection_point(self._lines[ii], self._lines[jj])
				if 0 <= point_[0] < max_w and 0 <= point_[1] < max_h:
					self._points.append(point_)
				jj += 1
			ii += 1

	def cross_check(self, crossing, tolerance):
		for p in self._points:
			num_node = -1
			i = 0
			while i < len(crossing):
				# if np.linalg.norm(p - np.array([crossing[i].data.column, crossing[i].data.middle])) < tolerance:
				if np.linalg.norm(p - crossing[i].data.middle) < tolerance:
					num_node = crossing[i].number_of_connected_node
					crossing.pop(i)
					if num_node == 3:
						break
					continue
				i += 1
			if num_node != -1:
				self.crossing.append((p,num_node))

	def get_point(self):
		return [Cross(c[0], c[1]) for c in self.crossing]

	def visualize_cross(self, img, circle_size = 1, color = [255, 0, 0]):
		for p in self._points:
			cv2.circle(img, (int(p[0]), int(p[1])), circle_size, color, -1)

	def get_points(self):
		return self._points

class Cross(object):
	L = 2
	T = 3
	X = 4
	Unknown = 100
	type_dict = {2:L, 3:T, 4:X, 100:Unknown}
	def __init__(self, coor, numnode):
		self.coordinate = coor
		self.type = self.type_dict[numnode] if self.type_dict.has_key(numnode) else self.type_dict[100]
