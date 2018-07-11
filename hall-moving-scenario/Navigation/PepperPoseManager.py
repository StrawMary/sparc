import numpy
import cv2
import math
import time

from naoqi import ALProxy
from PIL import Image
import qi

class PepperPoseManager:
	def __init__(self, IP="192.168.0.115", Port=9559):
		self.posture = ALProxy("ALRobotPosture", IP, Port)

	def stand_init(self):
		self.posture.goToPosture("StandInit", 2.0)

