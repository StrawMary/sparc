import config as cfg
import json
import rospy
import requests

from naoqi import ALProxy
from std_msgs.msg import String


class SpeechManager:
	def __init__(self, task_m):
		self.catch_phrase = cfg.speech_catch_phrase
		self.fade_duration = cfg.fade_duration
		self.robot_stream = cfg.robot_stream
		self.task_m = task_m

		self.activated = False

		if self.robot_stream:
			self.leds_service = ALProxy("ALLeds", cfg.ip, cfg.port)
			self.speech_service = ALProxy("ALTextToSpeech", cfg.ip, cfg.port)
		rospy.Subscriber('speech_text', String, self.callback)

	def run(self):
		if self.robot_stream:
			rospy.spin()

	def run_task_say(self, task):
		if self.robot_stream:
			self.speech_service.say(str(task.value), "English")

	def activate(self):
		self.activated = True

		if self.robot_stream:
			self.leds_service.fadeRGB('FaceLeds', 'green', self.fade_duration)

	def deactivate(self):
		self.activated = False

		if self.robot_stream:
			self.leds_service.fadeRGB('FaceLeds', "white", self.fade_duration)

	def interpret(self, text, language='en-EN'):
		if not text:
			return None

		params = {'access_token': cfg.access_keys[language], 'q': text}
		response = requests.get(url=cfg.URL, params=params).json()
		if 'entities' not in response:
			print('API server error: ' + str(response))
			return None
		entities = response['entities']
		if len(entities.keys()) < 2 or 'intent' not in entities or len(entities['intent']) == 0:
			print('Cannot interpret speech.')
			return None
		intent = entities['intent'][0]
		if cfg.intent_entities[intent['value']] not in entities or len(cfg.intent_entities[intent['value']]) == 0:
			print('Missing intent entity.')
			return None
		entity = entities[cfg.intent_entities[intent['value']]][0]

		return {'intent': intent['value'], 'entity': entity['value']}

	def callback(self, data):
		data = json.loads(data.data)
		if not data['text']:
			return
		text = data['text'].strip().lower()
		language = data['language']

		if text == self.catch_phrase:
			self.activate()
			return

		interpreted_speech = self.interpret(text, language)
		self.task_m(interpreted_speech)

		if self.activated:
			self.deactivate()


if __name__ == '__main__':
	class AttrDict(dict):
		def __init__(self, *args, **kwargs):
			super(AttrDict, self).__init__(*args, **kwargs)
			self.__dict__ = self
	data = AttrDict()
	data.update({'data': raw_input('Command:')})

	def test_f(text):
		print(text)
	speech_recognizer = SpeechManager(test_f)
	speech_recognizer.callback(data)
