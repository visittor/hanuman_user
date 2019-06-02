_DEFAULT_CONFIG = {
	'VelocityParameter' : { 'VelocityXWhenFollowTheBall_max' : '0.5',
							'VelocityXWhenFollowTheBall_min' : '0.1',
							'VelocityXWhenFollowTheBall_m'	 : '0.5',
							'VelocityXWhenFollowTheBall_c'	 : '0.0',

							'VelocityYWhenFollowTheBall' : '0.0',

							'OmegaZWhenRotateToBall_max'	 : '0.4',
							'OmegaZWhenRotateToBall_min'	 : '0.2',
							'OmegaZWhenRotateToBall_m'	 	 : '0.8',
							'OmegaZWhenRotateToBall_c'	 	 : '0.0',

							'VelocityXWhenStepToKick'	 : '1.0',
							'WaitingTimeAfterStep'		 : '3.0',

							'VelocityXSlideCurve'		 : '0.0',
							'VelocityYSlideCurve'		 : '0.8',
							'OmegaZSlideCurve'			 : '0.65',
							'WaitingTimeSlideCurve'		 : '3.0' },

	'ChangeStateParameter':{	'NearestDistanceFootballWrtRobot' 	: '0.04',
								'SmallDegreeToAlignTheBall' 		: '15',
								'BallConfidenceThreshold' 			: '0.5',
								
								'LimitTiltAngleDegree'  			: '60',
								'LimitPanAngleDegree'   			: '15', },

	'PanTiltPlanner' : {					
							'TiltAngleForLookAtFoot': '60',
							'ScanBallPattern' 		: basic_pattern_near
						}
}

def getParameters( config, *arg ):

	try:
		for key in arg:

			config = config[ key ]

		return config
	except KeyError:

		config = _DEFAULT_CONFIG
		for key in arg:

			config = config[ key ]

		return config


