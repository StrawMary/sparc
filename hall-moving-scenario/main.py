import sys
naoqi_path = '/home/sparc-308/workspace/stefania/pynaoqi-python2.7/lib/python2.7/site-packages'
sys.path.insert(0, naoqi_path)
sys.path.append('/home/sparc-308/Desktop/final_integration/sparc/hall-moving-scenario')

import qi
import cv2
import traceback

import config as cfg
from navigation.navigation import Navigation
from robot_interaction.speech_recognition_subscriber import SpeechRecognizer
from task_management.task_manager import TaskManager, TaskType
from utils.data_processor import DataProcessor
from vision.facenet.face_detector_recognizer import FaceNetDetector
from vision.image_provider.image_provider import ImageProvider
from vision.qrcodes_handler.qrcodes_handler import QRCodesHandler
from vision.segmentation.human_segmentation import Segmentation
from vision.tracking.tracker import Tracker
from vision.yolo2.people_detector import PeopleDetector


class Main(object):
	def __init__(self, app):
		self.people_detector = PeopleDetector()
		self.tracker = Tracker()
		self.segmentation = Segmentation()
		self.face_detector = FaceNetDetector()
		self.qrcodes_handler = QRCodesHandler()
		self.data_processor = DataProcessor()

		self.task_manager = TaskManager()

		self.navigation = Navigation()
		
		self.is_running = False

		super(Main, self).__init__()

		if cfg.robot_stream:
			app.start()
			self.image_provider = ImageProvider(cfg.ip, cfg.port, cfg.frameRate)
			self.image_provider.connect()
		else:
			self.camera = cv2.VideoCapture(-1)


	def run(self):
		print('Running...')

		self.is_running = True

		try:
			while self.is_running:
				if cfg.robot_stream:
					image, depth_image, camera_height, head_yaw, head_pitch = self.image_provider.get_cv_image()
					self.image_provider.release_image()
				else:
					_, image = self.camera.read()
					depth_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
					camera_height = 1.2
					head_yaw = 0
					head_pitch = 0

				people_3d_positions = []
				objects_3d_positions = []
				qrcodes_3d_positions = []

				# Use YOLO to detect people in frames.
				people_bboxes, people_scores, objects = self.people_detector.detect(image)
				self.data_processor.square_detections(people_bboxes)

				# Check for QR codes.
				qrcodes = self.qrcodes_handler.detect_QRcodes(image)

				# Compute people IDs.
				people_ids = self.tracker.update_ids(people_bboxes, people_scores)

				# Use FaceNet to detect and recognize faces in RGB image.
				faces = self.face_detector.detect(image)

				# Associate detected faces with detected people.
				people = self.data_processor.associate_faces_to_people(people_bboxes, faces)

				# Segment humans in detections.
				segmented_image, mask = self.segmentation.segment_people(image, people)

				# Compute distances.
				people_distances, people_depth_bboxes = self.data_processor.compute_people_distances(
					depth_image,
					mask,
					people
				)
				object_distances, objects_depth_bboxes = self.data_processor.compute_objects_distances(
					depth_image,
					objects
				)
				qrcodes_distances, qrcodes_depth_bboxes = self.data_processor.compute_objects_distances(
					depth_image,
					qrcodes
				)

				# Get objects 3D position relatively to the robot.
				people_3d_positions = self.data_processor.get_people_3d_positions(
					people,
					people_ids,
					people_distances,
					head_yaw,
					head_pitch,
					camera_height
				)
				objects_3d_positions = self.data_processor.get_objects_3d_positions(
					objects,
					object_distances,
					head_yaw,
					head_pitch,
					camera_height
				)
				qrcodes_3d_positions = self.data_processor.get_objects_3d_positions(
					qrcodes,
					qrcodes_distances,
					head_yaw,
					head_pitch,
					camera_height
				)

				tasks = self.task_manager.create_tasks(TaskType.SHOW_ON_MAP, people_3d_positions, 'people')
				tasks.extend(
					self.task_manager.create_tasks(TaskType.SHOW_ON_MAP, objects_3d_positions, 'objects')
				)
				tasks.extend(
					self.task_manager.create_tasks(TaskType.SHOW_ON_MAP, qrcodes_3d_positions, 'qrcodes')
				)
				self.navigation.add_tasks(tasks)
				

				# Show detections on RGB and depth images and display detections in RViz.
				self.navigation.show_map_positions()
				image = self.people_detector.draw_people_detections(image, people_bboxes, people_scores, people_ids, people_distances)
				image = self.people_detector.draw_object_detections(image, objects, object_distances)
				image = self.people_detector.draw_object_detections(image, qrcodes, qrcodes_distances)
				image = self.face_detector.draw_detections(image, faces)
				depth_image = self.data_processor.draw_squares(depth_image, people_depth_bboxes)
				cv2.imshow('Segmented', segmented_image)
				cv2.imshow('Depth', depth_image)
				cv2.imshow('Image', image)

				key = cv2.waitKey(1)
				if key == 27: # Esc key to stop
					print('Script interrupted by user, shutting down...')
					self.is_running = False
					if cfg.robot_stream:
						self.image_provider.disconnect()
				elif key >= 49 and key <= 57: # Keys from 1-9.
					self.navigation.move_to_goal(key - 49)

		except KeyboardInterrupt:
			print('Forced interruption by user, shutting down...')
			self.is_running = False
			if cfg.robot_stream:
				self.image_provider.disconnect()

		except Exception as e:
			print('Thrown exception: ' + str(e))
			traceback.print_exc()
			self.is_running = False
			if cfg.robot_stream:
				self.image_provider.disconnect()



if __name__ == '__main__':
	application = None

	if cfg.robot_stream:
		# Initialize robot's NAOqi framework.
		try:
			connection_url = 'tcp://' + ip + ':' + str(port)
			application = qi.Application(['Main', '--qi-url=' + connection_url])
		except RuntimeError:
			print('Can\'t connect to NAOqi at \'' + ip + ':'  + str(port) +'\'.\n'
				'Please check your script arguments. Run with -h option for help.')
			sys.exit(1)

	main = Main(application)
	main.run()
