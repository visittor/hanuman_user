import numpy as np

from scipy import spatial

YOffsetContour_10Pixel = 10
YOffsetContour_15Pixel = 15

DistanceThreshold = 8.0

def findGoal( ransacContours, marker ):
	''' findGoal function
	'''
	
	#	two line scan
	ransacContours1 = ransacContours.copy()
	ransacContours2 = ransacContours.copy()

	#	offset
	ransacContours1[ :, :, 1 ] = ransacContours1[ :, :, 1 ] + YOffsetContour_10Pixel
	ransacContours2[ :, :, 1 ] = ransacContours2[ :, :, 1 ] + YOffsetContour_15Pixel

	#	initial previous point
	previousPoint1 = ransacContours1[ 1 ]
	previousPoint2 = ransacContours2[ 1 ]
	
	#	initial change list
	changeList1 = list()
	changeList2 = list()

	#	scan two lines
	for point1, point2 in zip( ransacContours1[ 1:-1 ], ransacContours2[ 1:-1 ] ):
		
		#   get x, y
		x1 = max( 0, min(point1[ 0 ][ 0 ], marker.shape[1]-1 ))
		y1 = max( 0, min(point1[ 0 ][ 1 ], marker.shape[0]-1 ))

		x2 = max( 0, min(point2[ 0 ][ 0 ], marker.shape[1]-1 ))
		y2 = max( 0, min(point2[ 0 ][ 1 ], marker.shape[0]-1 ))

		prevX1 = max( 0, min(previousPoint1[0][0], marker.shape[1]-1 ))
		prevY1 = max( 0, min(previousPoint1[0][1], marker.shape[0]-1 ))

		prevX2 = max( 0, min(previousPoint2[0][0], marker.shape[1]-1 ))
		prevY2 = max( 0, min(previousPoint2[0][1], marker.shape[0]-1 ))
		
		#   get current marker and previous marker
		currentMarker1 = marker[ y1, x1 ]
		previousMarker1 = marker[ prevY1, prevX1 ]

		currentMarker2 = marker[ y2, x2 ]
		previousMarker2 = marker[ prevY2, prevX2 ]
		
		if currentMarker1 != previousMarker1:
			if previousMarker1 == 2:
				changeList1.append( point1 )

		if currentMarker2 != previousMarker2:
			if previousMarker2 == 2:
				changeList2.append( point2 )
		
		previousPoint1 = point1
		previousPoint2 = point2
	
	# #	scan two lines
	# for point in ransacContours2[ 1:-1 ]:
	
	# 	x = max( 0, min(point2[ 0 ][ 0 ], marker.shape[1]-1 ))
	# 	y = max( 0, min(point2[ 0 ][ 1 ], marker.shape[0]-1 ))

	# 	prevX = max( 0, min(previousPoint2[0][0], marker.shape[1]-1 ))
	# 	prevY = max( 0, min(previousPoint2[0][1], marker.shape[0]-1 ))
		
	# 	currentMarker = marker[ y, x ]
	# 	previousMarker = marker[ prevY, prevX ]
		
	# 	if currentMarker != previousMarker:
		 
	# 		if previousMarker == 2:
	# 			changeList2.append( point )
		
	# 	previousPoint2 = point
	
	#	change list to array for find distance
	changeArray1 = np.array( changeList1 ).reshape( -1, 2 )
	changeArray2 = np.array( changeList2 ).reshape( -1, 2 )

	#	find distance for grouping point
	if len( changeArray1 ) != 0 and len( changeArray2 ) != 0:
	
		distanceMatrix = spatial.distance.cdist( changeArray1, changeArray2 ) 
		
		minIdxAxis_0 = np.argmin( distanceMatrix, axis = 0 )
		minIdxAxis_1 = np.argmin( distanceMatrix, axis = 1 )

		goalPointList = list()
		
		#print minIdxAxis_0, minIdxAxis_1
		
		for i, j in enumerate( minIdxAxis_1 ):
			if distanceMatrix[ i, j ] < 8.0:
				x1, y1 = changeList1[ i ][ 0 ][ 0 ], changeList1[ i ][ 0 ][ 1 ]
				x2, y2 = changeList2[ j ][ 0 ][ 0 ], changeList2[ j ][ 0 ][ 1 ]

				goalPointList.append( ( x1, y1 ) )
				
		return goalPointList

	else:
		return []


def findGoal2( imageROIList, ransacContours, marker ):
	''' findGoal2 function
	'''
	pass

def findObstacle( obstacleContour ):
	''' findObstacle function
	'''
	pass
