import config as cfg
import heapq

from enum import Enum
from navigation.navigation_manager import NavigationManager, ClassType, KNOWN_LABELS
from navigation.pose_manager import PepperPoseManager
from robot_interaction.speech_recognition_subscriber import SpeechManager
from task import Task


class TaskType(Enum):
	GO_TO = 1
	SAY_SOMETHING = 2
	FIND_OBJECT = 3


class TaskManager:
	def __init__(self):
		self.ongoing_tasks = []
		self.speech_manager = SpeechManager(self.interpret_speech)
		self.navigation_manager = NavigationManager()
		self.pose_manager = PepperPoseManager()

	def interpret_speech(self, data):
		if not data:
			return

		print(data)

		if data['intent'] == cfg.GO_TO_INTENT:
			if self.create_tasks_go(data['entity']):
				self.create_tasks_say('Going to ' + data['entity'])
			else:
				self.create_tasks_say('Sorry I cannot do that!')
		elif data['intent'] == cfg.FIND_INTENT:
			if self.create_tasks_find(data['entity']):
				self.create_tasks_say('Finding ' + data['entity'])
			else:
				self.create_tasks_say('Sorry I cannot do that!')
		elif data['intent'] == cfg.SAY_INTENT:
			if data['entity'] in cfg.presentations:
				self.create_tasks_say(cfg.presentations[data['entity']])
			else:
				self.create_tasks_say(cfg.presentations['default'])

	def create_tasks_go(self, data):
		if data and self.navigation_manager.is_located(data):
			go_to_task = Task(type=TaskType.GO_TO,
							  priority=cfg.GO_TO_PRIOR,
							  label=data,
							  value=self.navigation_manager.get_coordinate_for_label(data))
			heapq.heappush(self.ongoing_tasks, (go_to_task.priority, go_to_task))
			return True
		return False

	def create_tasks_say(self, data):
		if data:
			say_task = Task(type=TaskType.SAY_SOMETHING,
							priority=cfg.SAY_PRIOR,
							value=data)
			heapq.heappush(self.ongoing_tasks, (say_task.priority, say_task))
			return True
		return False

	def create_tasks_find(self, data):
		if data and self.navigation_manager.is_located(data):
			class_type = ClassType.PERSON
			if data in KNOWN_LABELS:
				class_type = ClassType.OBJECT
			find_task = Task(type=TaskType.FIND_OBJECT,
							 class_type=class_type,
							 priority=cfg.FIND_PRIOR,
							 label=data,
							 value=self.navigation_manager.get_coordinate_for_label(data))
			heapq.heappush(self.ongoing_tasks, (find_task.priority, find_task))
			return True
		return False

	def get_next_task(self):
		if self.ongoing_tasks:
			return heapq.heappop(self.ongoing_tasks)
		return -1, None

	def step(self):
		prior, task = self.get_next_task()
		if task:
			self.execute_task(task)

	def execute_task(self, task):
		print("Executing task: " + str(task))
		if task.type == TaskType.GO_TO:
			self.navigation_manager.run_task_go_to(task)
		elif task.type == TaskType.SAY_SOMETHING:
			self.speech_manager.run_task_say(task)
		elif task.type == TaskType.FIND_OBJECT:
			self.navigation_manager.run_task_find(task)
