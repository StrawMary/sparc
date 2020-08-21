
class ChoregrapheBehaviour:
	def __init__(self, app):
		if app:
			self.behaviour_manager = app.session.service("ALBehaviorManager")
		self.running_behaviour = None
		self.on_success = None

	def on_finished_behaviour(self, data):
		if self.on_success:
			self.running_behaviour = None
			self.on_success()

	def play_behaviour(self, behaviour_name, on_success=None, on_fail=None):
		if self.behaviour_manager:
			self.running_behaviour = behaviour_name
			self.on_success = on_success
			promise = self.behaviour_manager.runBehavior(behaviour_name, _async=True)
			promise.addCallback(self.on_finished_behaviour)
		else:
			self.running_behaviour = None
			on_fail()

	def stop_behaviour(self):
		if self.running_behaviour:
			self.behaviour_manager.stopBehavior(self.running_behaviour)
			self.running_behaviour = None
