import config as cfg
import rospy
import time
import speech_recognition as sr

from std_msgs.msg import String

class SpeechRecognizer:
	def __init__(self):
		if cfg.robot_stream:
			rospy.Subscriber('speech', String, self.callback)
			rospy.init_node('listener', anonymous=True)

	def run(self):
		if cfg.robot_stream:
			rospy.spin()

	def callback(self, data):
		print('Recognized speech: ' + data.data)

if __name__ == '__main__':
	speech_recognizer = SpeechRecognizer()
	speech_recognizer.run()
