import config as cfg
import rospy

from naoqi import ALProxy
from std_msgs.msg import String


class SpeechManager:
	def __init__(self, task_m):
		self.catch_phrase = cfg.catch_phrase
		self.fade_duration = cfg.fade_duration
		self.robot_stream = cfg.robot_stream
		self.task_m = task_m

		self.leds_service = ALProxy("ALLeds", cfg.ip, cfg.port)
		self.speech_service = ALProxy("ALTextToSpeech", cfg.ip, cfg.port)

		self.activated = False

		if self.robot_stream:
			rospy.Subscriber('speech_text', String, self.callback)

	def run(self):
		if self.robot_stream:
			rospy.spin()

	def run_task_say(self, task):
		self.speech_service.say(task.value, "English")

	def activate(self):
		self.activated = True
		self.leds_service.fadeRGB('FaceLeds', 'green', self.fade_duration)

	def deactivate(self):
		self.activated = False
		self.leds_service.fadeRGB('FaceLeds', "white", self.fade_duration)

	def callback(self, data):
		text = data.data.strip().lower()
		print('Recognized speech: ' + text)

		if text == self.catch_phrase:
			self.activate()
			return

		self.task_m(text)

		if self.activated:
			self.deactivate()


if __name__ == '__main__':
	speech_recognizer = SpeechManager()
	speech_recognizer.run()
