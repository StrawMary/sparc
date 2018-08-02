import sys
naoqi_path = '/home/sparc-308/workspace/stefania/pynaoqi-python2.7/lib/python2.7/site-packages'
sys.path.insert(0, naoqi_path)

sys.path.append('/home/sparc-308/Desktop/final_integration/sparc/external_platform')

import qi
import config as cfg
import rospy
import signal
import traceback
import time

from task_management.task_manager import TaskManager
from vision.vision_manager import VisionManager


class Main(object):
	def __init__(self, app):
		self.vision_manager = VisionManager()
		self.task_manager = TaskManager()

		if cfg.robot_stream:
			app.start()
		rospy.init_node('listener', anonymous=True)

		super(Main, self).__init__()

	def interuppt_handler(self, signum, frame):
		print('Forced interruption by user, shutting down...')
		self.terminate()
		sys.exit(-2)

	def terminate(self):
		self.vision_manager.shut_down()
		self.task_manager.navigation_manager.shut_down()

	def run(self):
		print('Running...')
		if cfg.robot_stream:
			self.task_manager.pose_manager.stand_init()

		try:
			while self.vision_manager.is_running():
				if cfg.debug_mode:
					start_time = time.time()
				people_3d_positions, objects_3d_positions, qrcodes_3d_positions = self.vision_manager.detect()
				if cfg.debug_mode:
					vision_time = time.time()
					print("--- vision %s seconds ---" % (vision_time - start_time))
				self.task_manager.navigation_manager.show(people_3d_positions, objects_3d_positions, qrcodes_3d_positions)
				if cfg.debug_mode:
					display_time = time.time()
					print("--- rviz display %s seconds ---" % (display_time - vision_time))
				self.task_manager.step()
				if cfg.debug_mode:
					print("--- task execution %s seconds ---" % (time.time() - display_time))
					print("\n")
			self.terminate()

		except KeyboardInterrupt:
			print('Forced interruption by user, shutting down...')
			self.terminate()

		except Exception as e:
			print('Thrown exception: ' + str(e))
			traceback.print_exc()
			self.terminate()


if __name__ == '__main__':
	application = None

	if cfg.robot_stream:
		# Initialize robot's NAOqi framework.
		try:
			connection_url = 'tcp://' + cfg.ip + ':' + str(cfg.port)
			application = qi.Application(['Main', '--qi-url=' + connection_url])
		except RuntimeError:
			print('Can\'t connect to NAOqi at \'' + cfg.ip + ':'  + str(cfg.port) +'\'.\n'
				'Please check your script arguments. Run with -h option for help.')
			sys.exit(1)

	main = Main(application)
	signal.signal(signal.SIGINT, main.interuppt_handler)
	main.run()
