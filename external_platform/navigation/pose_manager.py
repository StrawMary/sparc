import config as cfg
import sys
sys.path.insert(0, cfg.naoqi_path)

from naoqi import ALProxy
import qi

class PepperPoseManager:
	def __init__(self, session=None):
		if cfg.robot_stream:
			self.posture = session.service('ALRobotPosture')
			self.tactile_gesture = session.service('ALTactileGesture')
			self.autonomous_life = session.service('ALAutonomousLife')

			self.signal = self.tactile_gesture.onGesture.connect(self.tactile_gesture_handler)

			self.waiting_for_gesture = False
			self.on_success = None

	def stand_init(self):
		if cfg.robot_stream:
			self.posture.goToPosture('StandInit', 0.5)

	def stand_serve(self):
		if cfg.robot_stream:
			self.posture.goToPosture('Serve', 0.5)

	def apply_pose(self, pose):
		if pose == "Serve":
			self.stand_serve()
		elif pose == "StandInit":
			self.stand_init()

	def stop(self):
		self.stand_init()

	def wait_for_gesture(self, on_success):
		self.waiting_for_gesture = True
		self.on_success = on_success

	def tactile_gesture_handler(self, value):
		if value and self.waiting_for_gesture:
			self.waiting_for_gesture = False
			self.on_success()
			self.on_success = None

	def stop_waiting(self):
		self.waiting_for_gesture = False
		self.on_success = None
 
