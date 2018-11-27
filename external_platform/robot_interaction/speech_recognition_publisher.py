import sys
sys.path.append('../')

import config as cfg
import sys
sys.path.insert(0, cfg.naoqi_path)

from naoqi import ALProxy
import json
import rospy
import speech_recognition as sr
import traceback
from std_msgs.msg import String
from pepper_microphone import PepperSpeechRecognitionEngine

audio_stream = True
robot_input = True

language = cfg.language


class SpeechRecognizer:
	def __init__(self):
		self.activated = True
		if robot_input:
			#self.leds_proxy = ALProxy('ALLeds', cfg.ip, cfg.port)
			rospy.init_node('mic_listener', anonymous=True)
			self.publisher = rospy.Publisher('speech_text', String, queue_size=10)

		if audio_stream:
			if not robot_input:
				self.microphone = sr.Microphone()
				self.engine = sr.Recognizer()
			else:
				self.pepper_engine = PepperSpeechRecognitionEngine(False)

		self.rate = rospy.Rate(10)

		print('Used language for recognition: ' + language)

	def recognize_speech(self):
		response = {
			'text': None,
			'error': None,
			'language': language
		}
		if audio_stream:
			if robot_input:
				print("Listening")
				#self.leds_proxy.fadeRGB('FaceLeds', "blue", 1)
				audio = self.pepper_engine.listen()
				#self.leds_proxy.fadeRGB('FaceLeds', "white", 1)
				print("Recognizing")

				response['text'] = self.pepper_engine.recognize_google(audio, language, False)
			else:
				with self.microphone as source:
					self.engine.adjust_for_ambient_noise(source)
					audio = self.engine.listen(source)
				try:
					response['text'] = self.engine.recognize_google(audio, language=language)
				except sr.RequestError:
					response['error'] = 'API unavailable'
				except sr.UnknownValueError:
					response['error'] = 'Unable to recognize speech'
		else:
			response['text'] = raw_input("Query: ")

		if response['text'] and response['text'].lower() == 'exit':
			exit(0)

		return response

	def run(self):
		try:
			while not rospy.is_shutdown():
				speech = self.recognize_speech()
				print(speech)
				if speech and speech['text']:
					self.publisher.publish(json.dumps(speech))
					self.rate.sleep()

		except KeyboardInterrupt:
			print('Forced interruption by user, shutting down...')

		except Exception as e:
			print('Thrown exception: ' + str(e))
			traceback.print_exc()

		except rospy.ROSInterruptException:
			print('ROS error')


if __name__ == '__main__':
	response = raw_input("Audio stream? [y/n]: ")
	if response == 'no' or response == 'n':
		audio_stream = False

	speech_recognizer = SpeechRecognizer()
	speech_recognizer.run()