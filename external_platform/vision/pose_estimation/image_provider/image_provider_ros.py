import cv2
import rospy

from collections import deque
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ImageMsg


class ImageProvider:
	def __init__(self):
		self.connected = True
		self.memory = None

		self.bridge = CvBridge()

		self.top_camera_subscriber = rospy.Subscriber('/pepper_robot/naoqi_driver/camera/front/image_raw', ImageMsg,
													  self.on_rgb_image_received)
		self.depth_camera_subscriber = rospy.Subscriber('/pepper_robot/naoqi_driver/camera/depth/image_raw',
														ImageMsg, self.on_depth_image_received)

		self.rgb_buffer = deque(maxlen=1)
		self.depth_buffer = deque(maxlen=1)

	def on_rgb_image_received(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			image = cv2.resize(image, (640, 480))
			self.rgb_buffer.append(image)
		except CvBridgeError, e:
			print("Error converting")
			print(e)

	def on_depth_image_received(self, data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "8UC1")
			image = cv2.normalize(image, image, 0, 255, cv2.NORM_MINMAX)
			image = cv2.resize(image, (640, 480))
			self.depth_buffer.append(image)
		except CvBridgeError, e:
			print("Error converting")
			print(e)

	def get_cv_image(self):
		if not self.connected:
			raise Exception('ROS node not connected.')

		if len(self.rgb_buffer) == 0 or len(self.depth_buffer) == 0:
			return [], []

		opencv_image = self.rgb_buffer.popleft()
		opencv_image_depth = self.depth_buffer.popleft()

		return opencv_image, opencv_image_depth

	def get_image(self):
		image, depth_image = self.get_cv_image()

		# 	_, image = self.camera.read()
		# 	image = cv2.resize(image, (vision_cfg.width, vision_cfg.height))
		# 	depth_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

		return image, depth_image

	def release_image(self):
		pass

	def disconnect(self):
		self.connected = False
