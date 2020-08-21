
class SharedMemory:
	def __init__(self):
		self.memory = {}

	def __str__(self):
		return repr(self) + str(self.memory)

	def add_key(self, key, value):
		self.memory[key] = value

	def delete_key(self, key):
		if key in self.memory:
			del self.memory[key]

	def get_value(self, key):
		if key in self.memory:
			return self.memory[key]
		else:
			return None

instance = SharedMemory()


