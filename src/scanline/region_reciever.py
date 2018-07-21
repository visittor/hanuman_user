from scanline import ScanLine
import numpy as np
import cv2
from util import *

class Region_reciver(object):

	def __init__(self):
		self.angle_threshold = 0.41
		self.size_ratio = 1.0

	def unite_region(self, regions, connected_color, axis):
		united_region = [ [ linklist(regions[j][i]) for i in range(len(regions[j])) ] for j in range(len(regions)) ]
		first = None
		last = None
		r_ = None
		for r1 in range(len(regions)):
			last = first
			for r2 in range(len(regions[r1])):
				if regions[r1][r2].color != connected_color:
					continue
				first = r2
				if r1 != 0:
					last = self.connect_region(last, r2, united_region, regions[r1], regions[r1-1], r1, axis)
				r_ = r2
		return united_region

	def connect_region(self, last, r, united_region, region, preregion, col, axis):
		axis = (axis+1)%2
		if last is None:
			return None

		while last < len(preregion) and preregion[last].stop[axis] < region[r].start[axis]:
			last += 1

		if last >= len(preregion):
			return None

		if preregion[last].start[axis] <= region[r].stop[axis] and preregion[last].color == region[r].color:
			if self.check(united_region[col-1][last], united_region[col][r])==1 and united_region[col-1][last].is_tail():
				self.link_linklist(united_region[col][r], united_region[col-1][last])

		last_ = last + 1
		while last_ < len(preregion) and preregion[last_].start[axis] <= region[r].stop[axis]:
			if last_ >= len(preregion):
				return last

			if preregion[last_].stop[axis] >= region[r].start[axis] and preregion[last_].color == region[r].color:
				if self.check(united_region[col-1][last_], united_region[col][r])==1 and united_region[col-1][last_].is_tail():
					self.link_linklist(united_region[col][r], united_region[col-1][last_])
			else:
				return last
			last_ += 1
		return last

	def find_node(self, regions, connected_color, axis):
		# nodes = [ [Node((i+1)*(j+1) - 1, regions[j][i]) for i in range(len(regions[j]))] for j in range(len(regions))]
		nodes = []
		count = 0
		lenght = 0
		for i in range(len(regions)):
			nodes.append([])
			lenght += len(regions[i])
			for j in range(len(regions[i])):
				nodes[i].append(Node(count, regions[i][j]))
				count += 1
		first = None
		last = None
		r_ = None
		for r1 in range(len(regions)):
			last = first
			first = 0
			for r2 in range(len(regions[r1])):
				if regions[r1][r2].color != connected_color:
					continue
				if r1 != 0:
					last = self.connect_node(last, r2, nodes,regions[r1], regions[r1-1], r1, axis)
				r_ = r2
		# return united_region,[ j for i in nodes for j in i]
		return [ l for k in nodes for l in k if l.data.color == connected_color]

	def connect_node(self, last, r, nodes, region, preregion, col, axis):
		# axis = (axis+1)%2
		if last is None:
			return None
		# print "yeah", preregion[last].stop, region[r].start
		while last < len(preregion) and preregion[last].stop[axis] > region[r].start[axis]:
			last += 1
		if last >= len(preregion):
			return None
		if preregion[last].start[axis] >= region[r].stop[axis] and preregion[last].color == region[r].color:
			nodes[col][r].add_connected_node(nodes[col-1][last])
			nodes[col-1][last].add_connected_node(nodes[col][r])
			# if self.check(united_region[col-1][last], united_region[col][r], axis)==1 and united_region[col-1][last].is_tail():
			# 	self.link_linklist(united_region[col][r], united_region[col-1][last])
		last_ = last + 1
		while last_ < len(preregion) and preregion[last_].start[axis] >= region[r].stop[axis]:
			if last_ >= len(preregion):
				return last
			if preregion[last_].stop[axis] <= region[r].start[axis]:
				if preregion[last_].color == region[r].color:
					nodes[col][r].add_connected_node(nodes[col-1][last_])
					nodes[col-1][last_].add_connected_node(nodes[col][r])
					# if self.check(united_region[col-1][last_], united_region[col][r], axis)==1 and united_region[col-1][last_].is_tail():
					# 	self.link_linklist(united_region[col][r], united_region[col-1][last_])
			else:
				return last
			last_ += 1
		return last

	def filter_node(self, nodes):
		i = 0
		while i < len(nodes):
			if nodes[i].is_alone():
				nodes.pop(i)
				continue
			i += 1

	def check(self, last, r, axis):
		if last.is_head() == 0:
			v1 = last.region - last.backward.region
			v2 = r.region - last.region
			if float(r.region.stop[axis] - r.region.start[axis]) == 0 or float(last.region.stop[axis] - last.region.start[axis]) == 0:
				return 0
			ratio1 = float(last.region.stop[axis] - last.region.start[axis])/float(r.region.stop[axis] - r.region.start[axis])
			ratio2 = float(r.region.stop[axis] - r.region.start[axis])/float(last.region.stop[axis] - last.region.start[axis])
			ratio = max(ratio1, ratio2)
			if angle_between(v1[:2], v2[:2]) < self.angle_threshold:
				if self.size_ratio is None:
					return 1
				elif ratio < 1.0 + self.size_ratio:
					return 1
				return 0
			else:
				return 0
		else:
			if float(r.region.stop[axis] - r.region.start[axis]) == 0 or float(last.region.stop[axis] - last.region.start[axis]) == 0:
				return 0
			ratio1 = float(last.region.stop[axis] - last.region.start[axis])/float(r.region.stop[axis] - r.region.start[axis])
			ratio2 = float(r.region.stop[axis] - r.region.start[axis])/float(last.region.stop[axis] - last.region.start[axis])
			ratio = max(ratio1, ratio2)
			if self.size_ratio is None:
				return 1
			elif ratio < 1.0 + self.size_ratio:
				return 1
			return 0

	def link_list_to_list(self, united_region):
		point_array = []
		for ii in united_region:
			temp = np.zeros((0,2), dtype = np.int)
			if ii.is_head() and ii.is_alone() == 0:
				temp = np.append(temp, [ii.region.middle], axis = 0)
				ii_ = ii.forward
				while ii_.is_tail() == 0:
					temp = np.append(temp, [ii_.region.middle], axis = 0)
					ii_ = ii_.forward
				temp = np.append(temp, [ii_.region.middle], axis = 0)
				if temp.shape[0] > 1:
					point_array.append((temp, ii.region.color))
		return point_array

	def link_linklist(self, forward, backward):
		forward.backward = backward
		backward.forward = forward

	def visualize_united_region(self, img, united_region, axis):
		for i in united_region:
			if i.is_tail() == 0:
				start = i.region
				stop = i.forward.region
				# p1 = (start.column,start.start) if axis == 1 else (start.start,start.column)
				# p2 = (stop.column,stop.start) if axis == 1 else (stop.start,stop.column)
				# p3 = (start.column,start.stop) if axis == 1 else (start.stop,start.column)
				# p4 = (stop.column,stop.stop) if axis == 1 else (stop.stop,stop.column)
				p1 = tuple(start.start)
				p2 = tuple(stop.start)
				p3 = tuple(start.stop)
				p4 = tuple(stop.stop)
				cv2.line(img, p1, p2, (0,0,255), 2)
				cv2.line(img, p3, p4, (0,0,255), 2)
				if i.is_head():
					cv2.line(img, p1, p3, (0,0,255), 2)
				if i.forward.is_tail():
					cv2.line(img, p2, p4, (0,0,255), 2)

	def visualize_node(self, img, nodes, axis, circle_size = 3, color = [255, 0, 255]):
		for n in nodes:
			p = tuple(n.data.middle)
			cv2.circle(img, p, circle_size * n.number_of_connected_node, color, -1)
			for cn in n.connected_node:
				p1 = tuple(cn.data.middle)
				p2 = tuple(n.data.middle)
				cv2.line(img, p1, p2, color, 2)

	def set_size_ratio(self, size_ratio):
		if size_ratio is None:
			self.size_ratio = None
		elif size_ratio < 0:
			self.size_ratio = -size_ratio
		else:
			self.size_ratio = size_ratio

	def set_angle_threshold(self, angle_threshold):
		self.angle_threshold = angle_threshold

class linklist(object):
	def __init__(self, region):
		self._region = region
		self._next = None
		self._previous = None

	def is_head(self):
		return 1 if self._previous is None else 0

	def is_tail(self):
		# if self._next is not None and self._next._previous is None:
		# 	print "maybe this a bug."
		return 1 if self._next is None else 0

	def is_alone(self):
		return 1 if self._next is None and self._previous is None else 0

	@property
	def forward(self):
		return self._next

	@forward.setter
	def forward(self, x):
		if self._next is None:
			self._next = x

	@property
	def backward(self):
		return self._previous

	@backward.setter
	def backward(self, x):
		if self._previous is None:
			self._previous = x

	@property
	def region(self):
		return self._region

class Node(object):
	def __init__(self, id_, data):
		self.id = id_
		self.data = data
		self.connected_node = []
		self.connected_node_id = []

	def is_alone(self):
		return 1 if len(self.connected_node_id) == 0 else 0

	def add_connected_node(self, node):
		if node.id not in self.connected_node_id:
			self.connected_node.append(node)
			self.connected_node_id.append(node.id)

	@property
	def number_of_connected_node(self):
		return len(self.connected_node_id)