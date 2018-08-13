import config as cfg
import heapq

from enum import Enum
from navigation.navigation_manager import NavigationManager
from navigation.pose_manager import PepperPoseManager
from robot_interaction.speech_recognition_subscriber import SpeechManager
from task_management.task import TaskStatus, MoveToTask, SayTask, SearchPersonTask, ShowRemindersTask
from task_management.behaviors import *
from vision.vision_manager import VisionManager


class TaskType(Enum):
	GO_TO = 1
	SAY_SOMETHING = 2
	FIND_OBJECT = 3


class TaskManager:
	def __init__(self, app):
		self.ongoing_tasks = []
		self.vision_manager = VisionManager(app)
		self.speech_manager = SpeechManager(app, self.create_behavior)
		self.navigation_manager = NavigationManager()
		self.pose_manager = PepperPoseManager()
		self.current_task = None

	def create_behavior(self, data):
		if data['intent'] == cfg.STOP_INTENT:
			self.stop_task(self.current_task)
			return

		if data['intent'] == cfg.SEARCH_INTENT:
			behavior_head = get_search_behavior(self, data['mandatory_entities'][0])
		elif data['intent'] == cfg.GO_TO_INTENT:
			behavior_head = get_go_to_behavior(self, data['mandatory_entities'][0])
		elif data['intent'] == cfg.FIND_INTENT:
			behavior_head = get_find_behavior(self, data['mandatory_entities'][0])
		elif data['intent'] == cfg.SAY_INTENT:
			if data['mandatory_entities'][0] in cfg.presentations:
				response = cfg.presentations[data['mandatory_entities'][0]]
				if callable(response):
					response = response(data['optional_entities'])
				behavior_head = get_say_behavior(self, response)
			else:
				behavior_head = get_say_behavior(self, cfg.presentations['default'])
		elif data['intent'] == cfg.HELLO_INTENT:
			behavior_head = get_say_behavior(self, cfg.hello_response)
		else:
			return

		self.add_task_to_queue(behavior_head)

	def create_task_say(self, text):
		if text:
			task = SayTask(self.speech_manager.say_async,
						   self.speech_manager.stop_async,
						   None,
						   None,
						   self.add_task_to_queue,
						   cfg.SAY_PRIOR,
						   text)
			return task
		return None

	def create_task_search(self, target):
		if target:
			task = SearchPersonTask(self.vision_manager.search_target,
									self.vision_manager.stop_search,
									None,
									None,
									self.add_task_to_queue,
									cfg.SEARCH_PRIOR,
									target)
			return task
		return None

	def create_task_go_to(self, target):
		if target and self.navigation_manager.is_located(target):
			task = MoveToTask(self.navigation_manager.move_to_coordinate,
							  self.navigation_manager.stop_movement,
							  None,
							  None,
							  self.add_task_to_queue,
							  cfg.GO_TO_PRIOR,
							  self.navigation_manager.get_coordinate_for_label(target))
			return task
		return None

	def create_task_show_reminders(self, url):
		if url:
			task = ShowRemindersTask(None,
									 None,
									 None,
									 None,
									 self.add_task_to_queue,
									 cfg.SHOW_REMINDERS_PRIOR,
									 url)
			return task
		return None

	def add_task_to_queue(self, task):
		heapq.heappush(self.ongoing_tasks, (task.priority, task))

	def execute_task(self, task):
		print("Executing task: " + str(task))
		task.run()

	def stop_task(self, task):
		if task:
			task.stop()

	def get_next_task(self):
		if self.ongoing_tasks and len(self.ongoing_tasks) > 0:
			if not self.current_task or self.current_task.priority > self.ongoing_tasks[0][0]:
				return heapq.heappop(self.ongoing_tasks)
		return -1, None

	def step(self):
		if self.current_task and \
				self.current_task.status != TaskStatus.PENDING and \
				self.current_task.status != TaskStatus.RUNNING:
			self.current_task = None

		prior, task = self.get_next_task()
		if not task:
			return
		if task != self.current_task:
			if self.current_task:
				self.stop_task(self.current_task)
				self.add_task_to_queue(self.current_task)
			self.current_task = task
			self.execute_task(self.current_task)
