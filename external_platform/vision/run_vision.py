import sys
sys.path.append('../')

import config as cfg
sys.path.insert(0, cfg.naoqi_path)

import argparse
import qi
import rospy
import signal
import traceback
import time

from vision.vision_manager import VisionManager


class Main(object):
	def __init__(self, app, robot_stream, debug_mode):
		self.debug_mode = debug_mode
		if robot_stream:
			rospy.init_node('camera_listener', anonymous=True)
			app.start()

		self.vision_manager = VisionManager(app, robot_stream, True, debug_mode)
		super(Main, self).__init__()

	def interuppt_handler(self, signum, frame):
		print('Forced interruption by user, shutting down...')
		self.terminate()
		sys.exit(-2)

	def terminate(self):
		self.vision_manager.shut_down()

	def run(self):
		print('Running...')

		try:
			while self.vision_manager.is_running():
				if self.debug_mode:
					start_time = time.time()
				(_, _, _) = self.vision_manager.detect()
				if self.debug_mode:
					print("Processing time: \t %s seconds" % (time.time() - start_time))
			self.terminate()

		except KeyboardInterrupt:
			print('Forced interruption by user, shutting down...')
			self.terminate()

		except Exception as e:
			print('Thrown exception: ' + str(e))
			traceback.print_exc()
			self.terminate()


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Run the vision component.')
	parser.add_argument("-r", "--robot_stream", help="Use the input from the robot's camera", action="store_true")
	parser.add_argument("-v", "--verbose", help="Display information about execution time", action="store_true")
	args = parser.parse_args()

	application = None
	if args.robot_stream:
		# Initialize robot's NAOqi framework.
		try:
			connection_url = 'tcp://' + cfg.ip + ':' + str(cfg.port)
			application = qi.Application(['Main', '--qi-url=' + connection_url])
		except RuntimeError:
			print('Can\'t connect to NAOqi at \'' + cfg.ip + ':'  + str(cfg.port) +'\'.\n'
				'Please check your script arguments. Run with -h option for help.')
			sys.exit(1)

	main = Main(application, args.robot_stream, args.verbose)
	signal.signal(signal.SIGINT, main.interuppt_handler)
	main.run()
