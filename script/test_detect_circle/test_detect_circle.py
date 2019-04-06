#!/usr/bin/python

import optparse
import configobj

import numpy as np
import cv2
import matplotlib.pyplot as plt 

from skimage import measure, feature

def findCircle( points, *args, **kwargs ):
	'''
	do something
	'''

	if len( points ) == 0:
		return 
	points = points[:,:2]
	model, inliers = measure.ransac(points, measure.CircleModel,
                        min_samples=3, residual_threshold=0.05,
                        max_trials=500, is_model_valid = None,
                        is_data_valid = None)

	if model is None:
		return

	x, y, r = model.params

	return x, y, r if np.sum( inliers ) > 5 else None

def visualize( point3D, circle = None, width = 1000 ):

	field = np.zeros( (width, width, 3), dtype = np.uint8 )
	field[:,:,1] = 255

	for ii in range( 0, width, 100 ):
		cv2.line( field, (ii, 0), (ii, width), (0,0,0), 1 )
		cv2.line( field, (0, ii), (width, ii), (0,0,0), 1 )

	pt1 = ( width / 2, 550 )
	pt2 = ( (width / 2)-15, width )
	pt3 = ( (width / 2)+15, width )

	triangle_cnt = np.array( [pt1, pt2, pt3] )

	# cv2.drawContours(field, [triangle_cnt], 0, (0, 0, 255), -1)
	# print self.points3D
	for x, y, z in point3D:

		x = int( x * 100.0 )
		y = int( y * 100.0 )
		y = (width / 2) - y
		x = width - x

		cv2.circle( field, (y,x), 3, (0,0,255), -1 )

	if circle is not None:
		x, y, r = circle

		if x is None or y is None or r is None:
			pass

		else:

			x = int(x*100.0)
			y = int(y*100.0)
			r = int(r*100.0)
			y = (width/2) - y
			x = width - x

			cv2.circle( field, (y,x), r, (255,0,0), 3 )

	return field

def main( ):
	parser = optparse.OptionParser( )

	parser.add_option( '-i', type = 'string', action = 'store', dest = 'input',
						help = 'Path to input.' )

	options, args = parser.parse_args( )

	if options.input is None:
		return

	data = np.load( options.input )

	point3DList = data[ 'point3D' ][1:]
	point3DList = [ np.vstack( p ) if len( p ) > 0 else p for p in point3DList ]

	point2DList = data[ 'point2D' ][1:]
	point2DList = [ np.vstack( p ) if len(p) > 0 else p for p in point2DList ]

	pantiltList = data[ 'pantilt' ][1:]
	timeStamp = data[ 'timeStamp' ][1:]
	robotDimension = data[ 'robotDimension' ][1:]

	# print timeStamp

	frame_indx = 0

	imgWidth = 1000

	is_stop = 0

	while True:
		print frame_indx+1, "/", len( point3DList )
		nextFrame_indx = (frame_indx + 1) % len( point3DList )

		difftime = timeStamp[ nextFrame_indx ] - timeStamp[ frame_indx ]

		circle = findCircle( point3DList[ frame_indx ] )

		img = visualize( point3DList[ frame_indx ], circle = circle, width = imgWidth )

		cv2.imshow( 'img', img )
		k = cv2.waitKey( 1 )

		if k == ord( 's' ):
			is_stop = ( is_stop + 1 )%2

		if is_stop:
			continue

		frame_indx = nextFrame_indx

if __name__ == '__main__':
	main( )

