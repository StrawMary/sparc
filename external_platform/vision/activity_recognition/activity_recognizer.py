import json
import rospy

from std_msgs.msg import String


class ActivityRecognizer:
	def __init__(self, callback_function, continuous_sending=True):
		self.manager_callback_function = callback_function
		self.continuous_sending = continuous_sending
		self.recognition_started = False
		self.poses_publisher = rospy.Publisher('/detected_poses', String, queue_size=10)
		self.poses_subscriber = rospy.Subscriber('/recognized_action', String, self.on_recognized_action)

	def update_people_poses(self, poses):
		if self.continuous_sending or self.recognition_started:
			self.poses_publisher.publish(json.dumps(poses))

	def on_recognized_action(self, data):
		if not self.recognition_started or not data:
			return
		recognized_action = data.data
		self.recognition_started = False
		self.manager_callback_function(recognized_action)

	def start_recognition(self):
		self.recognition_started = True

	def stop_recognition(self):
		self.recognition_started = False

