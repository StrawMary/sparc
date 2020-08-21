import sys
import argparse
sys.path.append('../')

import rospy

from natural_language_processor import NaturalLanguageProcessor
from speech_recognizer import SpeechRecognizer
from std_msgs.msg import String


class NLP:
	def __init__(self, audio_stream):
		self.speech_recognizer = SpeechRecognizer(audio_stream)
		self.natural_language_processor = NaturalLanguageProcessor()

	def run(self):
		self.speech_recognizer.run()


if __name__ == '__main__':
	def print_data(data):
		pass

	parser = argparse.ArgumentParser(description='Description of your program')
	parser.add_argument('-a','--audio', action='store_true')
	args = vars(parser.parse_args())

	rospy.init_node('mic_listener', anonymous=True)
	rospy.Subscriber('/commands_structured', String, print_data)

	use_audio_stream = False
	#user_response = raw_input("Audio stream? [y/n]: ")

	user_response = args['audio']
	if user_response:
		use_audio_stream = True

	nlp = NLP(use_audio_stream)
	nlp.run()
