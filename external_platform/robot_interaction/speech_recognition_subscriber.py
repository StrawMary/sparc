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

	def stop(self):
		self.speech_service.stopAll()

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

		# Entities should contain "intent", "target"(optional).
		if 'entities' not in response:
			print('API server error: ' + str(response))
			return None

		# Check that entities contain an intent.
		entities = response['entities']
		if 'intent' not in entities or len(entities['intent']) == 0:
			print('No intent found.')
			return None

		intent = entities['intent'][0]

		# Check that we have the mandatory entities to create a task.
		if not self.check_mandatory_entities(cfg.mandatory_intent_entities[intent['value']], entities):
			print('Missing required entity.')
			return None

		mandatory_entities = self.get_mandatory_entities(cfg.mandatory_intent_entities[intent['value']], entities)
		optional_entities = self.get_optional_entities(cfg.mandatory_intent_entities[intent['value']], entities)

		return {'intent': intent['value'], 'mandatory_entities': mandatory_entities, 'optional_entities': optional_entities}

	def check_mandatory_entities(self, mandatory_entities, received_entities):
		for entity in mandatory_entities:
			if entity not in received_entities:
				return False
		return True

	def get_mandatory_entities(self, mandatory_entities, received_entities):
		entities = []
		for mandatory_entity in mandatory_entities:
			for received_entity in received_entities[mandatory_entity]:
				entities.append(received_entity['value'])
		return entities

	def get_optional_entities(self, mandatory_entities, received_entities):
		entities = []
		for received_entity in received_entities.keys():
			if received_entity not in mandatory_entities and received_entity != 'intent':
				for entity in received_entities[received_entity]:
					entities.append(entity['value'])
		return entities

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
