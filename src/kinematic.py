from visionManager.visionModule import KinematicModule
from newbie_hanuman.msg import HanumanStatusMsg

from std_msgs.msg import Empty

class Kinematic(KinematicModule):

	def __init__(self):
		super(Kinematic, self).__init__()

		self.objectsMsgType = Empty
		self.posDictMsgType = Empty

		# TODO : uncomment lines below when we are ready.
		# self.motorCortexTopicName = '/spinalcord/hanuman_status'
		# self.motorCortexMsgType = HanumanStatusMsg

	def kinematicCalculation(self, objMsg, js, rconfig=None):
		return Empty()

	def loop(self):
		pass