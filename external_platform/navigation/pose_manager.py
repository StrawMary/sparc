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
		self.posture = ALProxy('ALRobotPosture', ip, port)

	def stand_init(self):
		self.posture.goToPosture('StandInit', 1.0)

