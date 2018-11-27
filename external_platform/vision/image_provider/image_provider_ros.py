import config as cfg
import cv2
import numpy as np
import rospy

from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from naoqi import ALProxy
from sensor_msgs.msg import Image as ImageMsg


class ImageProvider:
	def __init__(self):
		self.IP = cfg.ip
		self.PORT = cfg.port
		self.connected = False
		self.memory = None
		self.bridge = CvBridge()

		self.top_camera_subscriber = rospy.Subscriber('/pepper_robot/naoqi_driver/camera/front/image_raw', ImageMsg, self.on_rgb_image_received)
		self.depth_camera_subscriber = rospy.Subscriber('/pepper_robot/naoqi_driver/camera/depth/image_raw', ImageMsg, self.on_depth_image_received)

		self.rgb_buffer = deque(maxlen=1)
		self.depth_buffer = deque(maxlen=1)

	def on_rgb_image_received(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.rgb_buffer.append(image)
		except CvBridgeError, e:
			print("Error converting")
			print(e)

	def on_depth_image_received(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "8UC1")
			image = cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
			image = cv2.resize(image, (cfg.width, cfg.height))
			self.depth_buffer.append(image)
		except CvBridgeError, e:
			print("Error converting")
			print(e)

	def connect(self):
		self.memory = ALProxy('ALMemory', self.IP, self.PORT)
		self.connected = True

	def get_cv_image(self):
		if not self.connected:
			raise Exception('ROS node not connected.')

		if len(self.rgb_buffer) == 0 or len(self.depth_buffer) == 0:
			return [], [], None, None, None

		opencv_image = self.rgb_buffer.popleft()
		opencv_image_depth = self.depth_buffer.popleft()

		# Get yaw angle of the head in radians.
		head_yaw = self.memory.getData('Device/SubDeviceList/HeadYaw/Position/Sensor/Value')
		head_pitch = self.memory.getData('Device/SubDeviceList/HeadPitch/Position/Sensor/Value')

		return opencv_image, opencv_image_depth, cfg.top_camera_height, head_yaw, head_pitch

	def release_image(self):
		pass

	def disconnect(self):
		self.connected = False
