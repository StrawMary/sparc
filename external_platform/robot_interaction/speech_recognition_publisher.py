import json
import rospy
import speech_recognition as sr
import traceback
from std_msgs.msg import String

audio_stream = True

api_key = '026f72c808604bbeabaf4af3f9339334'
language_en = 'en-EN'
language_ro = 'ro-RO'

language = language_ro


class SpeechRecognizer:
	def __init__(self):
		self.engine = sr.Recognizer()
		self.activated = True

		if audio_stream:
			self.microphone = sr.Microphone()
		self.publisher = rospy.Publisher('speech_text', String, queue_size=10)
		rospy.init_node('talker', anonymous=True)
		self.rate = rospy.Rate(10)

		print('Used language for recognition: ' + language)

	def recognize_speech(self):
		response = {
			'text': None,
			'error': None,
			'language': language
		}

		if audio_stream:
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
			if response['text'] == 'exit':
				exit(0)

		return response

	def run(self):
		try:
			while not rospy.is_shutdown():
				speech = self.recognize_speech()
				if speech and speech['text']:
					print(speech)
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
