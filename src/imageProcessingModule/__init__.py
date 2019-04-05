import numpy as np

from scipy import spatial


def findGoal( ransacContours, marker ):
    
    #	
    #	two line scan
    #
    ransacContours1 = ransacContours.copy()
    ransacContours2 = ransacContours.copy()
    
    #	offset
    ransacContours1[ :, :, 1 ] = ransacContours1[ :, :, 1 ] + 10
    ransacContours2[ :, :, 1 ] = ransacContours2[ :, :, 1 ] + 15
    
    #	initial previous point
    previousPoint1 = ransacContours1[ 1 ]
    previousPoint2 = ransacContours2[ 1 ]
    
    #	initial change list
    changeList1 = list()
    changeList2 = list()
    
#		print "	ransac 1 "
    #	scan two lines
    for point in ransacContours1[ 1:-1 ]:
        
        x = point[ 0 ][ 0 ]
        y = point[ 0 ][ 1 ]
        
        currentMarker = marker[ y, x ]
        previousMarker = marker[ previousPoint1[ 0 ][ 1 ], previousPoint1[ 0 ][ 0 ] ]
        
        if currentMarker != previousMarker:
#				print "		current marker : {}".format( currentMarker )
#				print "		previous marker : {}".format( previousMarker )	
            
            if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 2:
                #isChange = True
                changeList1.append( point )
        
        previousPoint1 = point
    
#		print "	ransac 2 "
    
    #	scan two lines
    for point in ransacContours2[ 1:-1 ]:
    
        x = point[ 0 ][ 0 ]
        y = point[ 0 ][ 1 ]
        
        currentMarker = marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ]
        previousMarker = marker[ previousPoint2[ 0 ][ 1 ], previousPoint2[ 0 ][ 0 ] ]
        
        if currentMarker != previousMarker:
#				print "		current marker : {}".format( currentMarker )
#				print "		previous marker : {}".format( previousMarker )
            
            if marker[ point[ 0 ][ 1 ], point[ 0 ][ 0 ] ] == 2:
                #isChange = True
                changeList2.append( point )
        
        previousPoint2 = point
    
    #	change list to array for find distance
    changeArray1 = np.array( changeList1 ).reshape( -1, 2 )
    changeArray2 = np.array( changeList2 ).reshape( -1, 2 )
    #print changeList2
    #	find distance for grouping point
    if len( changeArray1 ) != 0 and len( changeArray2 ) != 0:
    
        distanceMatrix = spatial.distance.cdist( changeArray1, changeArray2 ) 
        minIdxAxis_0 = np.argmin( distanceMatrix, axis = 0 )
        minIdxAxis_1 = np.argmin( distanceMatrix, axis = 1 )

        goalPointList = list()
        
        #print minIdxAxis_0, minIdxAxis_1
        
        for i, j in enumerate( minIdxAxis_1 ):
            if distanceMatrix[ i, j ] < 5.5:
                x1, y1 = changeList1[ i ][ 0 ][ 0 ], changeList1[ i ][ 0 ][ 1 ]
                x2, y2 = changeList2[ j ][ 0 ][ 0 ], changeList2[ j ][ 0 ][ 1 ]

                goalPointList.append( ( x1, y1 ) )
                
        return goalPointList
        
#			if len( goalPointList ) == 4:
#								
#				goalLeft = goalPointList[ 0 : 2 ]
#				goalRight = goalPointList[ 2 : 4 ]
#				
#				return ( goalLeft, goalRight )
#				
#			else:
#				return ( None, None )
    #
    else:
        return [ None ]	