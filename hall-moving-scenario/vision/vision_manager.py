import config as cfg
import cv2

from vision.data_processor import DataProcessor
from vision.facenet.face_detector_recognizer import FaceNetDetector
from vision.image_provider.image_provider import ImageProvider
from vision.qrcodes_handler.qrcodes_handler import QRCodesHandler
from vision.segmentation.human_segmentation import Segmentation
from vision.tracking.tracker import Tracker
from vision.yolo2.people_detector import PeopleDetector


class VisionManager:
	def __init__(self):
		self.people_detector = PeopleDetector()
		self.tracker = Tracker()
		self.segmentation = Segmentation()
		self.face_detector = FaceNetDetector()
		self.qrcodes_handler = QRCodesHandler()
		self.data_processor = DataProcessor()
		self.running = True

		if cfg.robot_stream:
			self.image_provider = ImageProvider(cfg.ip, cfg.port, cfg.frameRate)
			self.image_provider.connect()
		else:
			self.camera = cv2.VideoCapture(-1)

	def detect(self):
		if cfg.robot_stream:
			image, depth_image, camera_height, head_yaw, head_pitch = self.image_provider.get_cv_image()
			self.image_provider.release_image()
		else:
			_, image = self.camera.read()
			depth_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
			camera_height = 1.2
			head_yaw = 0
			head_pitch = 0

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

		if cfg.debug_mode:
			# Show detections on RGB and depth images and display detections in RViz.
			image = self.people_detector.draw_people_detections(image, people_bboxes, people_scores, people_ids,
																people_distances)
			image = self.people_detector.draw_object_detections(image, objects, object_distances)
			image = self.people_detector.draw_object_detections(image, qrcodes, qrcodes_distances)
			image = self.face_detector.draw_detections(image, faces)
			depth_image = self.data_processor.draw_squares(depth_image, people_depth_bboxes)
			cv2.imshow('Segmented', segmented_image)
			cv2.imshow('Depth', depth_image)
			cv2.imshow('Image', image)
			key = cv2.waitKey(1)
			if key == 27:  # Esc key to stop
				print('Script interrupted by user, shutting down...')
				self.running = False

		return people_3d_positions, objects_3d_positions, qrcodes_3d_positions

	def shut_down(self):
		if cfg.robot_stream:
			self.image_provider.disconnect()
		else:
			self.camera.release()

		if cfg.debug_mode:
			cv2.destroyAllWindows()

	def is_running(self):
		return self.running
