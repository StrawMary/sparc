import rospy
import speech_recognition as sr
import time
import traceback
import pickle
from std_msgs.msg import String


api_key = '026f72c808604bbeabaf4af3f9339334'


class SpeechRecognizer:
	def __init__(self):
		self.engine = sr.Recognizer()
		self.activated = True

		self.microphone = sr.Microphone()
		self.publisher = rospy.Publisher('speech', String, queue_size=10)
		rospy.init_node('talker', anonymous=True)
		self.rate = rospy.Rate(10)

	def recognize_speech(self):
		response = {
			'text': None,
			'error': None,
		}

		with self.microphone as source:
			self.engine.adjust_for_ambient_noise(source)
			audio = self.engine.listen(source)

		try:
			response['text'] = self.engine.recognize_bing(audio, key=api_key)
		except sr.RequestError:
			response['error'] = 'API unavailable'
		except sr.UnknownValueError:
			response['error'] = 'Unable to recognize speech'

		return response

	def run(self):
		try:
			while not rospy.is_shutdown():
				speech = self.recognize_speech()
				print(speech)
				if not speech['error'] and speech['text']:
					if(speech['text'].lower() == 'hey pepper'):
						self.activated = True
						print("Listening")
						continue

				if self.activated:
					self.publisher.publish(str(speech['text']))
					self.rate.sleep()
					self.activated = False

		except KeyboardInterrupt:
			print('Forced interruption by user, shutting down...')

		except Exception as e:
			print('Thrown exception: ' + str(e))
			traceback.print_exc()

		except rospy.ROSInterruptException:
			print('ROS error')


if __name__ == '__main__':
	speech_recognizer = SpeechRecognizer()
	speech_recognizer.run()
