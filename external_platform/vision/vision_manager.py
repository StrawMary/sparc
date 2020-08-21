import config as cfg
import cv2
import numpy as np
import os
import random
import threading
import time
import vision_config as vision_cfg
from shared_memory import instance

import sys
sys.path.append('./pose_estimation/')


from vision.activity_recognition.activity_recognizer import ActivityRecognizer
from vision.data_processor import DataProcessor
from vision.facenet.face_detector_recognizer import FaceNetDetector
from vision.facenet.train_network import *
from vision.image_provider.image_provider_ros import ImageProvider
from vision.pose_estimation.pose_estimator import PoseEstimator
from vision.qrcodes_handler.qrcodes_handler import QRCodesHandler
#from vision.segmentation.human_segmentation import Segmentation
from vision.deep_tracking.object_tracker import ObjectTracker
from vision.yolo3.object_detector import ObjectDetector as ObjectDetectorV3


class VisionManager:
	def __init__(self, app, robot_stream=cfg.robot_stream, display_images=cfg.display_images, debug=cfg.debug_mode):
		self.robot_stream = robot_stream
		self.display_images = display_images
		self.debug_mode = debug

		self.object_detector = ObjectDetectorV3()
		self.tracker = ObjectTracker()
		#self.segmentation = Segmentation()
		self.face_detector = FaceNetDetector()
		self.qrcodes_handler = QRCodesHandler()
		self.data_processor = DataProcessor()
		self.pose_estimator = PoseEstimator()
		self.activity_recognizer = ActivityRecognizer(self.on_recognized_activity, continuous_sending=False)
		self.image_provider = ImageProvider(robot_stream)
		self.running = True

		self.face_detector_mutex = threading.Lock()

		if self.robot_stream:
			self.behavior_manager = app.session.service("ALBehaviorManager")

		self.searched_activity = None
		self.save_action_recognition_result_method = None
		self.searched_target = None
		self.found_searched_target = False
		self.target_to_remember = None
		self.result_key = None
		self.on_success = None
		self.on_fail = None
		self.on_going_say_promise = None
		self.on_going_say_promise_canceled = False
		self.timer = None

		self.threading_result = {'yolo': [],
								 'facenet': [],
								 'openpose': []}
		self.threads = []

	def get_thread_result(self, net_type, image, depth_image):
		if net_type == 'yolo':
			people, objects = self.object_detector.detect(image)
			self.data_processor.square_detections(people)
			#segmented_image, mask = self.segmentation.segment_people_bboxes(image, people)
			self.threading_result[net_type] = (people, objects, depth_image, depth_image)
		elif net_type == 'facenet':
			self.face_detector_mutex.acquire()
			self.threading_result[net_type] = self.face_detector.detect(image)
			self.face_detector_mutex.release()
		elif net_type == 'openpose':
			self.threading_result[net_type] = self.pose_estimator.estimate_poses(image, depth_image)

	def detect(self):
		start_time = time.time()

		image, depth_image, camera_height, head_yaw, head_pitch = self.image_provider.get_image()
		if len(image) == 0:
			return [], [], []

		# Run neural nets in parallel.
		for net_type in self.threading_result:
			thread = threading.Thread(target=self.get_thread_result, args=[net_type, image, depth_image])
			thread.start()
			self.threads.append(thread)

		for thread in self.threads:
			thread.join()
		self.threads = []

		nets_time = time.time()
		#print("\t Neural nets: \t %s seconds" % (nets_time - start_time))

		# Use YOLO to detect people in frames.
		people, objects, segmented_image, mask = self.threading_result['yolo']

		# Use OpenPose to estimate poses in frames.
		poses, display_poses = self.threading_result['openpose']
		self.activity_recognizer.update_people_poses(poses)

		# Check for QR codes.
		qrcodes = self.qrcodes_handler.detect_QRcodes(image)

		# Compute people IDs.
		people_ids = self.tracker.update_ids(image, people)

		# Use FaceNet to detect and recognize faces in RGB image.
		faces = self.threading_result['facenet']

		# Associate detected faces with detected people.
		people_data = self.data_processor.associate_faces_to_people(people, faces)

		# Compute distances.
		people_distances, people_depth_bboxes = self.data_processor.compute_people_distances(
			depth_image,
			mask,
			people_data
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
			people_data,
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

		pos_time = time.time()
		#print("\t 3D positions: \t %s seconds" % (pos_time - nets_time))

		#self.check_hand_up(display_poses, depth_image, camera_height, head_yaw, head_pitch)
		self.check_detected_target(people_data, qrcodes, objects)
		self.try_remember_target(image, people_data, qrcodes, objects)

		#print("Total time: \t %s seconds" % (time.time() - start_time))

		if self.display_images:
			# Show detections on RGB and depth images and display detections in RViz.
			image = self.object_detector.draw_people_detections(image, people, people_ids,
																people_distances)
			image = self.object_detector.draw_object_detections(image, objects, object_distances)
			image = self.object_detector.draw_object_detections(image, qrcodes, qrcodes_distances)
			image = self.pose_estimator.draw_poses(image, display_poses)
			self.face_detector_mutex.acquire()
			image = self.face_detector.draw_detections(image, faces)
			image = self.face_detector.draw_emotions(image, faces)
			self.face_detector_mutex.release()
			depth_image = self.data_processor.draw_squares(depth_image, people_depth_bboxes)
			#cv2.imshow('Segmented', segmented_image)
			cv2.imshow('Depth', depth_image)
			cv2.imshow('Image', image)
			key = cv2.waitKey(1)
			if key == 27:  # Esc key to stop
				print('Script interrupted by user, shutting down...')
				self.running = False

		#return people_3d_positions, objects_3d_positions, qrcodes_3d_positions
		return people_3d_positions, [], qrcodes_3d_positions

	def shut_down(self):
		if self.image_provider:
			self.image_provider.disconnect()

		if self.debug_mode:
			cv2.destroyAllWindows()

	def is_running(self):
		return self.running

	def clear_attrs(self):
		self.searched_target = None
		self.target_to_remember = None
		self.found_searched_target = False
		self.result_key = None
		self.on_success = None
		self.on_fail = None
		self.on_going_say_promise = None
		self.on_going_say_promise_canceled = False

	def get_3d_position_for_pose(self, pose, depth_image, camera_height, head_yaw, head_pitch):
		positions = []
		for joint in pose:
			if joint is not None:
				distance = self.data_processor.get_point_distance(joint, depth_image)
				if distance != 0.4 and distance != 5.0:
					angles = self.data_processor.compute_point_angles(joint)
					positions.append(self.data_processor.get_3d_position(distance, angles, head_yaw, head_pitch, camera_height))
		position = np.mean(positions, axis=0)
		return (position[0], position[1], 0)

	def check_hand_up(self, poses, depth_image, camera_height, head_yaw, head_pitch):
		if self.searched_target == "hand_up":
			for _, pose in poses:
				print(self.pose_estimator.check_hand_up(pose))
				if self.pose_estimator.check_hand_up(pose):
					self.found_searched_target = True
					if self.result_key:
						instance.add_key(self.result_key, self.get_3d_position_for_pose(pose, depth_image, camera_height, head_yaw, head_pitch))
					self.stop_search(external_stop=False)
					return

	def check_detected_target(self, people, qrcodes, objects):
		if self.searched_target:
			for person in people:
				if 'name' in person and person['name'].lower() == self.searched_target.lower():
					self.found_searched_target = True
					self.stop_search(external_stop=False)
					return
			for qrcode in qrcodes:
				if qrcode[1].lower() == self.searched_target.lower():
					self.found_searched_target = True
					self.stop_search(external_stop=False)
					return
			for obj in objects:
				if obj[2].lower() == self.searched_target.lower():
					self.found_searched_target = True
					self.stop_search(external_stop=False)
					return

	def move_head_callback(self, data):
		if data.hasError():
			self.on_fail()
		if not self.on_going_say_promise_canceled:
			if self.found_searched_target:
				self.on_success()
			else:
				self.on_fail()
		self.clear_attrs()

	def search_target(self, target, result_key, on_success=None, on_fail=None):
		if not target:
			on_fail()

		if cfg.robot_stream:
			self.searched_target = target
			self.result_key = result_key
			self.on_success = on_success
			self.on_fail = on_fail

			self.on_going_say_promise = self.behavior_manager.runBehavior(vision_cfg.move_head, _async=True)
			self.on_going_say_promise.addCallback(self.move_head_callback)
		else:
			self.timer = threading.Timer(5, self.on_timeout, [on_success, on_fail])
			self.timer.start()

	def on_timeout(self, on_success, on_fail):
		self.timer = None
		if random.random() < 0.5:
			if on_success:
				on_success()
		else:
			if on_fail:
				on_fail()

	def stop_search(self, external_stop=True):
		if self.robot_stream:
			if self.on_going_say_promise:
				self.on_going_say_promise_canceled = external_stop
				if self.behavior_manager and self.behavior_manager.isBehaviorRunning(vision_cfg.move_head):
					self.behavior_manager.stopBehavior(vision_cfg.move_head)
				if external_stop:
					# Stand_init()
					pass
		else:
			if self.timer:
				self.timer.cancel()
			self.timer = None

	def try_remember_target(self, image, people, qrcodes, objects):
		if self.target_to_remember:
			[target, target_type, images_no] = self.target_to_remember
			if images_no == 0:
				self.target_to_remember = None
				t = threading.Thread(target=self.retrain_recognition_network, args=(target_type,))
				t.start()
				return

			if target_type == "person":
				closest_face = None
				closest_face_dimension = 0
				for person in people:
					if "face_bbox" in person:
						left, top, right, bottom = person["face_bbox"]
						face_dimension = (right - left) * (bottom - top)
						if face_dimension > closest_face_dimension:
							closest_face = person["face_bbox"]
							closest_face_dimension = face_dimension

				if closest_face:
					left, top, right, bottom = closest_face
					left -= 10
					top -= 10
					right += 10
					bottom += 10
					cropped_image = image[top:top+(bottom-top), left:left+(right-left)]
					self.save_image(cropped_image, target, images_no)
					self.target_to_remember[2] -= 1
			else:
				if self.on_fail():
					self.on_fail()
				self.clear_attrs()

	def remember_target(self, target, target_type="person", on_success=None, on_fail=None):
		if not target:
			if on_fail:
				on_fail()
		self.on_success = on_success
		self.on_fail = on_fail
		self.target_to_remember = [target, target_type, vision_cfg.remember_images_no]

	def stop_remember(self):
		self.clear_attrs()

	def save_image(self, image, directory_name, image_no):
		path = os.path.join(cfg.new_people_path, directory_name)
		if not os.path.isdir(path):
			os.mkdir(path)

		cv2.resize(image, (int(len(image) *1.5), int(len(image[0])*1.5)))
		cv2.imwrite(os.path.join(path, str(image_no) + '.png'), image)

	def retrain_recognition_network(self, target_type):
		if target_type != "person":
			if self.on_fail:
				self.on_fail()
		else:
			train_network("new_faces_classifier")
			new_face_detector = FaceNetDetector("new_faces_classifier")
			self.face_detector_mutex.acquire()
			self.face_detector = new_face_detector
			self.face_detector_mutex.release()
			if self.on_success:
				self.on_success()

		self.clear_attrs()

	def recognize_activity(self, activity, save_result=None, on_success=None, on_fail=None):
		print("Start recognizing activity")
		if cfg.robot_stream:
			self.searched_activity = activity
			self.save_action_recognition_result_method = save_result
			self.on_success = on_success
			self.on_fail = on_fail

			self.activity_recognizer.start_recognition()
		else:
			self.timer = threading.Timer(5, self.on_timeout, [on_success, on_fail])
			self.timer.start()

	def stop_recognizing_activity(self):
		print("Stopped recognizing activity")
		self.activity_recognizer.stop_recognition()
		self.searched_activity = None

	def on_recognized_activity(self, activity):
		print(activity)
		if self.save_action_recognition_result_method:
			self.save_action_recognition_result_method(activity)
		self.activity_recognizer.stop_recognition()
		if self.searched_activity is not None and self.searched_activity != activity:
			self.searched_activity = None
			self.save_action_recognition_result_method = None
			if self.on_fail:
				self.on_fail()
		else:
			if self.on_success:
				self.on_success()

