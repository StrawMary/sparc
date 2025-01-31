import heapq
import robot_interaction.tablet_interaction_config as tablet_cfg
import task_management.tasks_config as tasks_cfg
import speech.speech_config as speech_cfg

from enum import Enum
from environment_interaction.environment_manager import EnvironmentManager
from navigation.navigation_manager import NavigationManager
from navigation.pose_manager import PepperPoseManager
from robot_interaction.choregraphe_behaviour_manager import ChoregrapheBehaviour
from robot_interaction.tablet_manager import TabletManager
from task_management.behaviors import *
from task_management.commands_processor import CommandsProcessor
from task_management.task import *
from speech.speech_manager import SpeechManager
from vision.vision_manager import VisionManager
from shared_memory import instance



class TaskType(Enum):
	GO_TO = 1
	SAY_SOMETHING = 2
	FIND_OBJECT = 3


class TaskManager:
	def __init__(self, app):
		self.ongoing_tasks = []
		self.current_task = None

		self.behaviour_manager = ChoregrapheBehaviour(app)
		self.environment_manager = EnvironmentManager()
		self.navigation_manager = NavigationManager(app.session if app is not None else None)
		self.speech_manager = SpeechManager(app)
		self.tablet_manager = TabletManager(app, self.speech_manager.say_async)
		self.vision_manager = VisionManager(app)

		self.commands_manager = CommandsProcessor(self.create_behavior)

	def create_behavior(self, data):
		if data['intent'] == speech_cfg.STOP_INTENT:
			self.stop_task(self.current_task)
			self.navigation_manager.pose_manager.stand_init()
			return

		if data['intent'] == speech_cfg.NEXT_INTENT or data['intent'] == speech_cfg.PREVIOUS_INTENT:
			self.tablet_manager.on_interaction_intent(data['intent'])
			return

		if data['intent'] == speech_cfg.SEARCH_INTENT:
			behavior_head = get_search_behavior(self, data['mandatory_entities']['target'])
		elif data['intent'] == speech_cfg.SERVE_INTENT:
			behavior_head = get_serve_behaviour(self)
		elif data['intent'] == speech_cfg.GO_TO_INTENT:
			behavior_head = get_go_to_behavior(self, data['mandatory_entities']['target'])
		elif data['intent'] == speech_cfg.FIND_INTENT:
			behavior_head = get_find_behavior(self, data['mandatory_entities']['target'])
		elif data['intent'] == speech_cfg.REMINDERS_INTENT:
			behavior_head = get_reminders_behavior(self, data['mandatory_entities']['target'])
		elif data['intent'] == speech_cfg.HEALTH_INTENT:
			behavior_head = get_health_behaviour(self, data['mandatory_entities']['health_entity'])
		elif data['intent'] == speech_cfg.ACTUATION_INTENT:
			behavior_head = get_actuators_behaviour(self, data['mandatory_entities']['target'], data['optional_entities'])
		elif data['intent'] == speech_cfg.REMEMBER_INTENT:
			behavior_head = get_remember_behaviour(self, {})
			#behavior_head = get_remember_behaviour(data['optional_entities'])
		elif data['intent'] == speech_cfg.SAY_INTENT:
			if data['mandatory_entities']['target'] in speech_cfg.presentations:
				response = speech_cfg.presentations[data['mandatory_entities']['target']]
				if callable(response):
					response = response(data['optional_entities'])
				behavior_head = get_say_behavior(self, response)
			elif data['mandatory_entities']['target'] == 'projects':
				behavior_head = get_demo_behaviour2(self, {})
			else:
				behavior_head = get_say_behavior(self, speech_cfg.presentations['default'])
		elif data['intent'] == speech_cfg.HELLO_INTENT:
			behavior_head = get_say_behavior(self, speech_cfg.hello_response)
		elif data['intent'] == speech_cfg.RECOGNIZE_ACTIVITY_INTENT:
			behavior_head = get_activity_test_behaviour(self, {})
		else:
			return

		if behavior_head:
			self.add_task_to_queue(behavior_head)

	def create_task_say(self, text):
		if text:
			task = SayTask(self.speech_manager.say_async,
						   self.speech_manager.stop_async,
						   None,
						   None,
						   self.add_task_to_queue,
						   tasks_cfg.SAY_PRIOR,
						   text)
			return task
		return None

	def create_task_listen(self, keywords=[]):
		task = ListenTask(self.speech_manager.listen,
						self.speech_manager.stop_listen,
						None,
						None,
						self.add_task_to_queue,
						tasks_cfg.LISTEN_PRIOR,
						keywords)
		return task

	def create_task_search(self, target, result_key=None):
		if target:
			task = SearchTargetTask(self.vision_manager.search_target,
									self.vision_manager.stop_search,
									None,
									None,
									self.add_task_to_queue,
									tasks_cfg.SEARCH_PRIOR,
									target, result_key)
			return task
		return None

	def create_task_remember(self, target, target_type="person"):
		if target:
			task = RememberTargetTask(self.vision_manager.remember_target,
									  self.vision_manager.stop_remember,
									  None,
									  None,
									  self.add_task_to_queue,
									  tasks_cfg.REMEMBER_PRIOR,
									  target,
									  target_type)
			return task
		return None

	def create_task_go_to(self, target, offset = 0):
		if target.startswith("#") or target == 'last_pos':
			print("Creating task")
			task = MoveToTask(self.navigation_manager.move_to_memory_target,
							  self.navigation_manager.stop_movement,
							  None,
							  None,
							  self.add_task_to_queue,
							  tasks_cfg.GO_TO_PRIOR,
							  target,
							  offset)
			return task

		if target and self.navigation_manager.is_located(target):
			task = MoveToTask(self.navigation_manager.move_to_coordinate,
							  self.navigation_manager.stop_movement,
							  None,
							  None,
							  self.add_task_to_queue,
							  tasks_cfg.GO_TO_PRIOR,
							  self.navigation_manager.get_coordinate_for_label(target),
							  offset)
			return task
		return None

	def create_task_wait(self, target):
		if target:
			task = WaitTask(self.navigation_manager.wait_task,
							self.navigation_manager.stop_waiting,
							None,
							None,
							self.add_task_to_queue,
							tasks_cfg.POSE_PRIOR,
							target)
			return task
		return None

	def create_task_show_reminders(self, target):
		if target:
			task = ShowURLTask(self.tablet_manager.display_reminders,
							   self.tablet_manager.clear_display,
							   None,
							   None,
							   self.add_task_to_queue,
							   tasks_cfg.SHOW_REMINDERS_PRIOR,
							   self.tablet_manager.get_url_for_person(target))
			return task
		return None

	def create_task_show_health_measurements(self, target):
		if target in tablet_cfg.HEALTH_MEASUREMENTS_URL:
			task = ShowURLTask(self.tablet_manager.display_health,
							   self.tablet_manager.clear_display,
							   None,
							   None,
							   self.add_task_to_queue,
							   tasks_cfg.HEALTH_PRIOR,
							   self.tablet_manager.get_url_for_target(target))
			return task
		return None

	def create_task_show_URL(self, target):
		if target:
			task = ShowURLTask(self.tablet_manager.display_webpage,
							   self.tablet_manager.clear_display,
							   None,
							   None,
							   self.add_task_to_queue,
							   tasks_cfg.SHOW_REMINDERS_PRIOR,
							   target)
			return task
		return None

	def create_task_actuation(self, target, optional_entities):
		if not target:
			return None

		task = ActuationTask(self.environment_manager.process_command,
							 self.environment_manager.stop,
							 None,
							 None,
							 self.add_task_to_queue,
							 tasks_cfg.ACTUATION_PRIOR,
							 target,
							 optional_entities)
		return task

	def create_task_pose(self, pose):
		if not pose:
			return None

		task = PoseTask(self.navigation_manager.set_pose,
						self.navigation_manager.stop_pose_movement,
						None,
						None,
						self.add_task_to_queue,
						tasks_cfg.POSE_PRIOR,
						pose)
		return task

	def create_task_choregraphe_behaviour(self, behaviour_name):
		if not behaviour_name:
			return None

		task = ChoregrapheBehaviourTask(self.behaviour_manager.play_behaviour,
									self.behaviour_manager.stop_behaviour,
									None,
									None,
									self.add_task_to_queue,
									tasks_cfg.CHOREGRAPHE_BEHAVIOUR_PRIOR,
									behaviour_name)
		return task

	def create_task_recognize_activity(self, activity=None):
		task = RecognizeActivityTask(self.vision_manager.recognize_activity,
									self.vision_manager.stop_recognizing_activity,
									None,
									None,
									self.add_task_to_queue,
									tasks_cfg.RECOGNIZE_ACTIVITY_PRIOR,
									activity)
		return task

	def add_task_to_queue(self, task):
		heapq.heappush(self.ongoing_tasks, (task.priority, task))
		print("Queue: " + str(self.ongoing_tasks))

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
		if not self.current_task and len(self.ongoing_tasks) == 0:
			self.navigation_manager.pose_manager.stand_init()
		if not task:
			return
		if task != self.current_task:
			if self.current_task:
				self.stop_task(self.current_task)
				if not hasattr(self.current_task, 'url'):
					self.add_task_to_queue(self.current_task)
			self.current_task = task
			self.execute_task(self.current_task)