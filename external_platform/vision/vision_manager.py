import config as cfg
import cv2
import time

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
		if cfg.debug_mode:
			start_time = time.time()
		if cfg.robot_stream:
			image, depth_image, camera_height, head_yaw, head_pitch = self.image_provider.get_cv_image()
			self.image_provider.release_image()
		else:
			_, image = self.camera.read()
			depth_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
			camera_height = 1.2
			head_yaw = 0
			head_pitch = 0

		if cfg.debug_mode:
			aq_time = time.time()
			print("\t image getter: \t %s seconds" % (aq_time - start_time))

		# Use YOLO to detect people in frames.
		people_bboxes, people_scores, objects = self.people_detector.detect(image)
		self.data_processor.square_detections(people_bboxes)

		if cfg.debug_mode:
			yolo_time = time.time()
			print("\t\t yolo: \t %s seconds" % (yolo_time - aq_time))

		# Segment humans in detections.
		segmented_image, mask = self.segmentation.segment_people_bboxes(image, people_bboxes)

		if cfg.debug_mode:
			segmentation_time = time.time()
			print("\t\t segmentation: \t %s seconds" % (segmentation_time - yolo_time))

		# Check for QR codes.
		qrcodes = self.qrcodes_handler.detect_QRcodes(image)

		if cfg.debug_mode:
			qr_time = time.time()
			print("\t\t qr codes: \t %s seconds" % (qr_time - segmentation_time))

		# Compute people IDs.
		people_ids = self.tracker.update_ids(people_bboxes, people_scores)

		if cfg.debug_mode:
			track_time = time.time()
			print("\t\t sort: \t %s seconds" % (track_time - qr_time))

		# Use FaceNet to detect and recognize faces in RGB image.
		faces = self.face_detector.detect(image)

		if cfg.debug_mode:
			facenet_time = time.time()
			print("\t\t facenet: \t %s seconds" % (facenet_time - track_time))

		# Associate detected faces with detected people.
		people = self.data_processor.associate_faces_to_people(people_bboxes, faces)

		if cfg.debug_mode:
			association_time = time.time()
			print("\t\t association: \t %s seconds" % (association_time - facenet_time))
			nets_time = time.time()
			print("\t neural nets: \t %s seconds" % (nets_time - aq_time))

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
			pos_time = time.time()
			print("\t 3d positions: \t %s seconds" % (pos_time - nets_time))

		if cfg.display_images:
			# Show detections on RGB and depth images and display detections in RViz.
			image = self.people_detector.draw_people_detections(image, people_bboxes, people_scores, people_ids,
																people_distances)
			image = self.people_detector.draw_object_detections(image, objects, object_distances)
			image = self.people_detector.draw_object_detections(image, qrcodes, qrcodes_distances)
			image = self.face_detector.draw_detections(image, faces)
			# depth_image = self.data_processor.draw_squares(depth_image, people_depth_bboxes)
			# cv2.imshow('Segmented', segmented_image)
			# cv2.imshow('Depth', depth_image)
			cv2.imshow('Image', image)
			key = cv2.waitKey(1)
			if key == 27:  # Esc key to stop
				print('Script interrupted by user, shutting down...')
				self.running = False

			if cfg.debug_mode:
				print("\t display image: \t %s seconds" % (time.time() - pos_time))

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
