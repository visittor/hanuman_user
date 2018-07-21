import numpy as np 

def unit_vector(vector):
	""" return unit_vector of vector"""
	return vector.astype(float)/np.linalg.norm(vector)

def angle_between(vec1, vec2):
	u_vec1 = unit_vector(vec1)
	u_vec2 = unit_vector(vec2)
	return np.arccos( np.clip(np.dot(u_vec1, u_vec2), -1, 1))

def frechet_distance(m1, m2):
	max_dis = 0
	for i in m1:
		min_dis = -1
		for j in m2:
			if min_dis == -1:
				min_dis = np.linalg.norm(i - j)
			elif np.linalg.norm(i - j) < min_dis:
				min_dis = np.linalg.norm(i - j)
		if min_dis > max_dis :
			max_dis = min_dis
	return max_dis

def find_line_intersection(m1, c1, m2, c2):
	a = np.array([ [m1, -1], [m2, -1] ])
	b = np.array([ -c1, -c2])
	return np.linalg.solve(a,b)

def classify_color(val, max_, min_):
	if (min_ < val).all() and (max_ > val).all():
		return 1
	return 0