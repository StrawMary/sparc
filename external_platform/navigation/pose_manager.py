import numpy
import cv2
import math
import time
import config as cfg

from naoqi import ALProxy
from PIL import Image
import qi

class PepperPoseManager:
	def __init__(self, ip=cfg.ip, port=cfg.port):
		if cfg.robot_stream:
			self.posture = ALProxy('ALRobotPosture', ip, port)

	def stand_init(self):
		if cfg.robot_stream:
			self.posture.goToPosture('StandInit', 1.0)

