from scanline import ScanLine
from region_reciever import Region_reciver
import numpy as np
import cv2
from util import *
import cyutility
import math

class Line_provider(Region_reciver):

	def __init__(self, angle_threshold = 0.51, size_ratio=1.0):
		super( Line_provider, self).__init__()
		super( Line_provider, self).set_size_ratio(size_ratio)
		super( Line_provider, self).set_angle_threshold(angle_threshold)
		self._united_region = []
		self.nodes = []
		self._region = None
		self.__grid_dis = None
		self.lines = []
		self.crossing = []

	def receive_region(self, region, connected_color = 0, axis = 1):
		self._region = region
		self.nodes = super(Line_provider, self).find_node(self._region, connected_color, axis)
		# self._united_region = super(Line_provider, self).unite_region(self._region, connected_color, axis)

	def visualize_united_region(self, img, axis = 1):
		super( Line_provider, self).visualize_node(img, self.nodes, axis)

	def visualize_node(self, img, axis = 1):
		super( Line_provider, self).visualize_node(img, self.nodes, axis)

	def link_list_to_list(self):
		return super( Line_provider, self ).link_list_to_list(self._united_region)

	def node_to_list(self, nodes):
		# visited_nodes = np.zeros(len(nodes), dtype = np.uint8)
		# for i in nodes:
		# 	print i.id
		visited_nodes = []
		line_array = []
		point_array = []
		i = 0
		while len(visited_nodes) != len(nodes):
			if nodes[i].id in visited_nodes:
				pass
				# print "pass"
			elif nodes[i].number_of_connected_node == 1:
				point_array.append(nodes[i])
				visited_nodes.append(nodes[i].id)
				# print "1 node"
				self.search_node(nodes[i], visited_nodes, line_array, point_array)
				i += 1
			elif nodes[i].number_of_connected_node == 2:
				if self.check_node_angle(nodes[i]):
					point_array.append(nodes[i])
					visited_nodes.append(nodes[i].id)
					# print "2 	node"
					self.search_node(nodes[i], visited_nodes, line_array, point_array)
			else:
				point_array.append(nodes[i])
				visited_nodes.append(nodes[i].id)
				# print "3 node"
				self.search_node(nodes[i], visited_nodes, line_array, point_array)
			i = (i+1)%len(nodes)
		# print line_array
		return point_array, line_array

	def search_node(self, node, visited_nodes, line_array, point_array):
		for next_node in node.connected_node:
			if next_node.id in visited_nodes:
				continue
			line_array.append([np.zeros((0,2), dtype = np.int), node.data.color])
			while next_node.id not in visited_nodes:
				if next_node.number_of_connected_node == 1:
					point_array.append(next_node)
					visited_nodes.append(next_node.id)
					break
				elif next_node.number_of_connected_node == 2:
					if self.check_node_angle(next_node):
						point_array.append(next_node)
						visited_nodes.append(next_node.id)
						self.search_node(next_node, visited_nodes, line_array, point_array)
						break
					else:
						line_array[-1][0] = np.append(line_array[-1][0], [next_node.data.middle], axis = 0)
						visited_nodes.append(next_node.id)
						next_node = next_node.connected_node[0] if next_node.connected_node[0].id not in visited_nodes else next_node.connected_node[1]
				else:
					point_array.append(next_node)
					visited_nodes.append(next_node.id)
					self.search_node(next_node, visited_nodes, line_array, point_array)
					break

	def check_node_angle(self, node):
		v1 = node.connected_node[0].data - node.data
		v2 = node.connected_node[1].data - node.data
		ang = angle_between(v1[:2], v2[:2])
		if ang < 3.14 - self.angle_threshold :
			return 1
		return 0

	def to_line_eq(self):
		crossing,point_array = self.node_to_list(self.nodes)
		# lines_ = []
		lines_ = np.zeros([len(point_array),4], dtype=np.float)
		i = 0
		for p,color in point_array:
			x = p[:,0]
			y = p[:,1]
			A = np.vstack([x, np.ones(len(x))]).T
			if len(A) == 0:
				continue
			try:
				m, c = np.linalg.lstsq(A, y)[0]
				# lines_.append([m,c,x[0],x[-1],color,p.shape[0]])
				lines_[i,0] = m
				lines_[i,1] = c
				lines_[i,2] = x[0]
				lines_[i,3] = x[-1]
				i += 1
			except ValueError:
				pass
		return lines_[:i], crossing

	def compare_lines(self, l1, l2, frechet_d_thr = 15):
		m1 = [np.array([ 0, l1[1] ]), np.array([100, 100*l1[0]+l1[1] ]) ]
		m2 = [np.array([ 0, l2[1] ]), np.array([100, 100*l2[0]+l2[1] ]) ]
		frechet_d = frechet_distance(m1, m2)
		if frechet_d < frechet_d_thr:
			return 1
		return 0 

	def merge_lines(self, l1, l2):
		l1[0] = (l1[5]*l1[0] + l2[5]*l2[0])/(l1[5]+l2[5])
		l1[1] = (l1[5]*l1[1] + l2[5]*l2[1])/(l1[5]+l2[5])
		l1[2] = l1[2] if l1[2] < l2[2] else l2[2]
		l1[3] = l1[3] if l1[3] > l2[3] else l2[3]
		l1[5] += l2[5]

	def filter_line(self, frechet_d_thr = 15):
		lines_,self.crossing = self.to_line_eq()
		# self.lines = []
		# while len(lines_) > 0:
		# 	line = lines_.pop(0)
		# 	ii = 0
		# 	while ii < len(lines_):
		# 		if self.compare_lines(line, lines_[ii], frechet_d_thr = frechet_d_thr):
		# 			self.merge_lines(line, lines_[ii])
		# 			lines_.pop(ii)
		# 		else:
		# 			ii += 1
		# 	if line[5] > 3:
		# 		self.lines.append(line)
		# self.lines = lines_
		# print lines_
		self.lines = cyutility.PyGroupingLineMC(lines_.astype(np.float), float(frechet_d_thr))

	def make_line(self, frechet_d_thr = 15):
		self.filter_line( frechet_d_thr = frechet_d_thr)

	def get_lines_array(self, frechet_d_thr = 15):
		self.filter_line( frechet_d_thr = frechet_d_thr)
		return self.lines

	def get_lines(self, axis = 1, frechet_distance = 15):
		self.filter_line( frechet_d_thr= frechet_distance)
		return [LineMC(l) for l in self.lines]

	@property
	def lenght(self):
		return len(self.lines)
		
	def __getitem__(self, index):
		return LineMC(self.lines[index])

class VerticleLine_provider(object):

	def __init__(self):
		self.verlineEqs = np.zeros((0,5), np.float)
		self.verlineList = []

	def scanImage(self, imgHSV, horizon = 0):
		if horizon != 0:
			skyHSV = imgHSV[:horizon, :, 2]
			skyHSV = np.float64(skyHSV) / 255.0
			sobel = np.absolute(cv2.Sobel(skyHSV, -1, 1, 0, scale=255)).astype(np.uint8)
			cv2.threshold(sobel, 127, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU, sobel)
			verLines = cv2.HoughLinesP(sobel,1,np.pi/2.0,100, horizon/50, 10)
			if verLines is not None:
				verLines = verLines.reshape(-1,4).astype(int)
				self.verlineEqs= np.zeros((len(verLines),5), np.float)
				cyutility.PyfromPoints2LineEq(verLines[:,1::-1], verLines[:,3:1:-1], self.verlineEqs, mode = 0)
				self.verlineEqs = cyutility.PyGroupingLineABC(self.verlineEqs, 20)
				self.__create_verlineList()
				return
		self.verlineEqs = np.zeros((0,5), np.float)
		self.verlineList = []

	def __create_verlineList(self):
		self.verlineList = [ LineABC(l) for l in self.verlineEqs if np.absolute(l[0]) < 2*np.absolute(l[1])]

	def get_verticalLine(self):
		return self.verlineList

	def scan_and_getLine(self, imgHSV, horizon = 0):
		self.scanImage(imgHSV, horizon)
		return self.verlineList

class LineMC(object):

	def __init__(self, line_array):
		self._line_array = line_array

	@property
	def m(self):
		return self._line_array[0]

	@property
	def c(self):
		return self._line_array[1]

	@property
	def start(self):
		y = self._line_array[0]*self._line_array[2] + self._line_array[1]
		return np.array([self._line_array[2], y])

	@property
	def stop(self):
		y = self._line_array[0]*self._line_array[3] + self._line_array[1]
		return np.array([self._line_array[3], y])

	@property
	def startX(self):
		return self._line_array[2]

	@property
	def stopX(self):
		return self._line_array[3]

	# @property
	# def color(self):
	# 	return self._line_array[4]

	# @property
	# def vote(self):
	# 	return self._line_array[5]

class LineABC(object):

	def __init__(self, line_array):
		self._line_array = line_array

	@property
	def A(self):
		return self._line_array[0]

	@property
	def B(self):
		return self._line_array[1]

	@property
	def C(self):
		return self._line_array[2]

	@property
	def start(self):
		if self._line_array[1] != 0:
			y = (-self._line_array[2] - (self._line_array[0]*self._line_array[3])) / self._line_array[1]
			y = int(y)
			return np.array([ self._line_array[3], y ])
		else:
			x = (-self._line_array[2] - (self._line_array[1]*self._line_array[3])) / self._line_array[0]
			x = int(x)
			return np.array([ x,self._line_array[3] ])

	@property
	def stop(self):
		if self._line_array[1] != 0:
			y = (-self._line_array[2] - (self._line_array[0]*self._line_array[4])) / self._line_array[1]
			y = int(y)
			return np.array([self._line_array[4], y])
		else:
			x = (-self._line_array[2] - (self._line_array[1]*self._line_array[4])) / self._line_array[0]
			x = int(x)
			return np.array([ x,self._line_array[4] ])

	@property
	def startX(self):
		return self._line_array[3]

	@property
	def stopY(self):
		return self._line_array[4]