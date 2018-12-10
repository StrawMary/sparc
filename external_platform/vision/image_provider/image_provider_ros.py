import config as cfg
import cv2
import rospy
import vision.vision_config as vision_cfg

from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from naoqi import ALProxy
from sensor_msgs.msg import Image as ImageMsg


class ImageProvider:
	def __init__(self):
		self.connected = True
		self.memory = None
		if cfg.robot_stream:
			self.bridge = CvBridge()

			self.top_camera_subscriber = rospy.Subscriber('/pepper_robot/naoqi_driver/camera/front/image_raw', ImageMsg,
														  self.on_rgb_image_received)
			self.depth_camera_subscriber = rospy.Subscriber('/pepper_robot/naoqi_driver/camera/depth/image_raw',
															ImageMsg, self.on_depth_image_received)
			self.memory = ALProxy('ALMemory', cfg.ip, cfg.port)

			self.rgb_buffer = deque(maxlen=1)
			self.depth_buffer = deque(maxlen=1)

		elif cfg.external_camera in vision_cfg.external_cameras:
			self.camera = cv2.VideoCapture(vision_cfg.external_cameras[cfg.external_camera])
			self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

		else:
			self.camera = cv2.VideoCapture(-1)
			self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
			image = cv2.resize(image, (vision_cfg.width, vision_cfg.height))
			self.depth_buffer.append(image)
		except CvBridgeError, e:
			print("Error converting")
			print(e)

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

		return opencv_image, opencv_image_depth, vision_cfg.top_camera_height, head_yaw, head_pitch

	def get_image(self):
		if cfg.robot_stream:
			image, depth_image, camera_height, head_yaw, head_pitch = self.get_cv_image()
		else:
			_, image = self.camera.read()
			image = cv2.resize(image, (vision_cfg.width, vision_cfg.height))
			depth_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
			camera_height = 1.2
			head_yaw = 0
			head_pitch = 0

		return image, depth_image, camera_height, head_yaw, head_pitch

	def release_image(self):
		pass

	def disconnect(self):
		self.connected = False
