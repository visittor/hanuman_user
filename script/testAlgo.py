import numpy as np
import cv2

from scanLine2 import findBoundary, findChangeOfColor
from colorSegmentation import colorSegmentation, createColorDefFromDict, waterShed

DEFAULT_COLORCONFIG = [
						{ 	'ID'		:	1,
							'Name'		:	'green',
							'H_max'		:	86,
							'S_max'		:	255,
							'V_max'		:	255,
							'H_min'		:	13,
							'S_min'		:	72,
							'V_min'		:	0,
							'RenderColor_RGB': '( 0, 128, 0 )',
							'MinArea_pixels' : 4 },
						{ 	'ID'		:	2,
							'Name'		:	'black',
							'H_max'		:	255,
							'S_max'		:	50,
							'V_max'		:	50,
							'H_min'		:	0,
							'S_min'		:	0,
							'V_min'		:	0,
							'RenderColor_RGB': '( 0, 0, 0 )',
							'MinArea_pixels' : 4 },
						{ 	'ID'		:	3,
							'Name'		:	'orange',
							'H_max'		:	255,
							'S_max'		:	255,
							'V_max'		:	255,
							'H_min'		:	255,
							'S_min'		:	255,
							'V_min'		:	255,
							'RenderColor_RGB': '( 255, 128, 0 )',
							'MinArea_pixels' : 4 },
						{ 	'ID'		:	4,
							'Name'		:	'blue',
							'H_max'		:	255,
							'S_max'		:	255,
							'V_max'		:	255,
							'H_min'		:	255,
							'S_min'		:	255,
							'V_min'		:	255,
							'RenderColor_RGB': '( 0, 0, 255 )',
							'MinArea_pixels' : 4 },
						{ 	'ID'		:	5,
							'Name'		:	'yellow',
							'H_max'		:	255,
							'S_max'		:	255,
							'V_max'		:	255,
							'H_min'		:	255,
							'S_min'		:	255,
							'V_min'		:	255,
							'RenderColor_RGB': '( 255, 255, 0 )',
							'MinArea_pixels' : 4 },
						{ 	'ID'		:	6,
							'Name'		:	'magenta',
							'H_max'		:	255,
							'S_max'		:	255,
							'V_max'		:	255,
							'H_min'		:	255,
							'S_min'		:	255,
							'V_min'		:	255,
							'RenderColor_RGB': '( 128, 0, 255 )',
							'MinArea_pixels' : 4 },
						{ 	'ID'		:	7,
							'Name'		:	'cyan',
							'H_max'		:	100,
							'S_max'		:	255,
							'V_max'		:	255,
							'H_min'		:	88,
							'S_min'		:	90,
							'V_min'		:	0,
							'RenderColor_RGB': '( 255, 255, 0 )',
							'MinArea_pixels' : 4 },
						{ 	'ID'		:	8,
							'Name'		:	'white',
							'H_max'		:	128,
							'S_max'		:	72,
							'V_max'		:	255,
							'H_min'		:	0,
							'S_min'		:	0,
							'V_min'		:	50,
							'RenderColor_RGB': '( 255, 255, 255 )',
							'MinArea_pixels' : 4 },
]

def renderColor( colorMap, colorDefList ):
	img = np.zeros( (colorMap.shape[0], colorMap.shape[1], 3), 
					dtype = np.uint8 )

	for colorDict in colorDefList:
		img[ colorMap[:,:] == colorDict.ID ] = eval( colorDict.RenderColor_RGB )

	return img

colorDefList = createColorDefFromDict( DEFAULT_COLORCONFIG )

cap = cv2.VideoCapture( 'field3.avi' )

stop = 0
ret, frame = cap.read( )

while True:

	if not stop:
		ret, frame = cap.read( )

	img = frame.copy()

	if not ret:
		print "Camera not found ..."
		break

	# hsv = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )

	img = cv2.GaussianBlur( img, ( 5, 5 ), 0 )

	colorMap = colorSegmentation( img, colorDefList )[:, : ]
	colorMap_watershed = waterShed( img, colorMap.copy() ).astype( np.uint8 )
	fieldContour, fieldMask = findBoundary( colorMap_watershed, 1 )
	pointClound = findChangeOfColor( colorMap_watershed, 8, 1, mask=fieldMask, step = 40 )

	# pointCloound = findChangeOfColor( colorMap, 8, 1, mask = fieldMask )

	renderedColorMap = renderColor( colorMap, colorDefList)
	renderedColorMap_watershed = renderColor( colorMap_watershed, colorDefList)
	cv2.drawContours( img, [ fieldContour ], 0, (0, 128, 255), 2 )

	for scanLine in pointClound:
		for x,y in scanLine:
			cv2.circle(img,(x,y), 4, (0,0,0), -1)
			cv2.circle(img,(x,y), 3, (0,0,255), -1)

	# cv2.imshow( 'colorMap', colorMap_watershed )
	# cv2.imshow( 'colorMap_watershed', renderedColorMap_watershed )
	cv2.imshow( 'originalImg', img )
	k = cv2.waitKey( 1 )

	if k == 27:
		break
	elif k == ord( 's' ):
		stop = ( stop + 1 ) % 2

cv2.destroyAllWindows()