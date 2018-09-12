import sys
sys.path.append('../')

import config as cfg
import json
import rospy
import requests
import threading

from std_msgs.msg import String


class SpeechManager:
	def __init__(self, app, on_speech_received, receive_commands=False):
		self.catch_phrase = cfg.speech_catch_phrase
		self.fade_duration = cfg.fade_duration
		self.robot_stream = cfg.robot_stream
		self.receive_commands = cfg.receive_commands or receive_commands
		self.on_speech_received = on_speech_received

		self.on_going_say_promise = None
		self.on_going_say_promise_canceled = False
		self.on_success = None
		self.on_fail = None
		self.timer = None

		if self.robot_stream and app:
			self.leds_service = app.session.service("ALLeds")
			self.speech_service = app.session.service("ALTextToSpeech")
		if self.robot_stream or self.receive_commands:
			self.recognition_subscriber = rospy.Subscriber('speech_text', String, self.callback_text)
			self.commands_subscriber = rospy.Subscriber('/commands_json', String, self.callback_json)


	def callback_text(self, received_data):
		data = json.loads(received_data.data)
		if not data['text']:
			return
		text = data['text'].strip().lower()
		language = data['language']

		response = self.call_wit(text, language)
		interpreted_speech = self.interpret(response)
		if interpreted_speech:
			self.on_speech_received(interpreted_speech)

	def callback_json(self, received_data):
		print(received_data.data)
		interpreted_speech = self.interpret(json.loads(received_data.data))
		if interpreted_speech:
			self.on_speech_received(interpreted_speech)

	def call_wit(self, text, language='en-EN'):
		if not text:
			return None

		params = {'access_token': cfg.access_keys[language], 'q': text}
		response = requests.get(url=cfg.URL, params=params).json()
		return response

	def interpret(self, response):
		# The response should contain entities associated with the query.
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

		interpreted_speech = {
			'intent': intent['value'],
			'mandatory_entities': mandatory_entities,
			'optional_entities': optional_entities
		}
		self.print_interpreted_speech(interpreted_speech)
		return interpreted_speech

	def print_interpreted_speech(self, interpreted_speech):
		print('\nInterpreted speech:')
		print('\tIntent: ' + str(interpreted_speech['intent']))
		print('\tEntities - mandatory: ' + str(interpreted_speech['mandatory_entities']))
		print('\t            optional: ' + str(interpreted_speech['optional_entities']))

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
		entities = {}
		for received_entity in received_entities.keys():
			if received_entity not in mandatory_entities and received_entity != 'intent':
				for entity in received_entities[received_entity]:
					entities[received_entity] = entity['value']
		return entities

	def clear_say_attrs(self):
		self.on_going_say_promise = None
		self.on_success = None
		self.on_fail = None
		self.on_going_say_promise_canceled = False

	def say_async_callback(self, data):
		if not self.on_going_say_promise_canceled:
			if not data or data.hasError():
				if self.on_fail:
					self.on_fail()
			else:
				if self.on_success:
					self.on_success()
		self.clear_say_attrs()

	def say_async(self, text, on_success=None, on_fail=None):
		if self.robot_stream:
			self.on_success = on_success
			self.on_fail = on_fail
			if text:
				text = text.encode("ascii", "ignore")
				self.on_going_say_promise = self.speech_service.say(str(text), "English", _async=True)
				self.on_going_say_promise.addCallback(self.say_async_callback)
			else:
				on_fail()
		else:
			self.timer = threading.Timer(5, self.on_timeout, [on_success])
			self.timer.start()

	def stop_async(self):
		if self.robot_stream:
			if self.on_going_say_promise:
				self.on_going_say_promise_canceled = True
				if self.speech_service:
					self.speech_service.stopAll()
		else:
			if self.timer:
				self.timer.cancel()
			self.timer = None

	def on_timeout(self, on_success):
		self.timer = None
		if on_success:
			on_success()


if __name__ == '__main__':
	def print_data(data):
		print(data)

	speech_subscriber = SpeechManager(None, print_data, True)