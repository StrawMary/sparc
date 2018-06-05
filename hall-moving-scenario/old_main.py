import sys
naoqi_path = "/home/sparc-308/workspace/stefania/pynaoqi-python2.7/lib/python2.7/site-packages"
sys.path.insert(0, naoqi_path)

import qi
import cv2
import pickle
import socket
import time
import traceback

import random

from FaceModules.FaceNet.face_detector_recognizer import FaceNetDetector
from ImageProvider.image_provider import ImageProvider
from PepperInteraction.reminders import RemindersChecker
from Segmentation.human_segmentation import HumanSegmentation
from TCPDataSender.data_sender import TCPDataSender
from Tracking.tracker import Tracker
from TrajectoryPredictors.trajectory_predictor import TrajectoryPredictor
from Yolo2.people_detector import PeopleDetector
from utils.data_processor import DataProcessor
from utils.distances_smoothening import KalmanSmoother

robot_stream = True
send_data = False
ip_fast = "192.168.0.115"
ip_local = "172.19.11.65"

ip = ip_fast
port = 9559
frameRate = 30

recognition_threshold = 0.5
trajectory_lags_no = 10
trajectory_predictor = 'arima'

class Main(object):
	def __init__(self, app):
		self.camera = cv2.VideoCapture(-1)
		self.people_detector = PeopleDetector()
		self.face_detector = FaceNetDetector("FaceModules/FaceNet", recognition_threshold)
		self.data_sender = TCPDataSender()
		self.data_processor = DataProcessor()
		self.human_segmentation = HumanSegmentation()
		self.image_provider = ImageProvider(ip, port, frameRate)
		self.tracker = Tracker()
		self.distances_smoothening = None

		self.positions_no = 0
		self.trajectories = {}

		super(Main, self).__init__()
		if robot_stream:
			app.start()
			self.image_provider.connect()


	def run(self):
		print("Waiting for connection...")
		if send_data:
			self.data_sender.start()
		print("Client connected.")

		try:
			while True:
				if robot_stream:
					image, depth_image, camera_height, head_yaw, head_pitch = self.image_provider.get_cv_image()
				else:
					_, image = self.camera.read()
					depth_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
					camera_height = 1.2
					head_yaw = 0
					head_pitch = 0

				# Use YOLO to detect people in the RGB image.
				start_time = time.time()
				people_bboxes, people_scores = self.people_detector.detect(image)
				if len(people_bboxes):
					print("--- %s yolo seconds ---" % (time.time() - start_time))
				self.data_processor.square_detections(people_bboxes)

				# Compute people IDs.
				people_ids = self.tracker.updateIds(people_bboxes, people_scores)

				# Use FaceNet to detect and recognize faces in RGB image.
				start_time = time.time()
				faces = self.face_detector.detect(image)
				if len(faces):
					print("--- %s facenet seconds ---" % (time.time() - start_time))

				# Associate detected faces with detected people.
				people = self.data_processor.associate_faces_to_people(people_bboxes, faces)


				start_time = time.time()
				# Segment humans in detections.
				segmented_image, mask = self.human_segmentation.get_segmented_image(image, people)
				if len(people_bboxes):
					print("--- %s segmentation seconds ---" % (time.time() - start_time))

				# Compute more information about detected people.
				people_angles = self.data_processor.compute_people_angles(people)
				people_distances, depth_bboxes = self.data_processor.compute_people_distances(depth_image, mask, people)
				

				if len(people_distances) > 0:
					if self.distances_smoothening == None:
						self.distances_smoothening = KalmanSmoother(people_distances[0])
					else:
						distance = self.distances_smoothening.get_distance_estimation(people_distances[0])
						if distance:
							people_distances[0] = distance
						else:
							people = []
							people_ids = []
							people_angles = []
							people_distances = []

				# Put all the informatin together.
				people_info = self.data_processor.get_people_info(people, people_ids, people_angles, people_distances)

				# Get people 3D position relative to the robot.
				people_3d_positions = self.data_processor.get_people_3d_positions(people_info, head_yaw, head_pitch, camera_height)

				# Send positions.
				if send_data:
					self.data_sender.send_data(pickle.dumps(people_3d_positions))
					#print(people_3d_positions)
					#self.data_sender.send_data(pickle.dumps([[0, [1 + random.random() / 10, 1 + random.random() / 10, 1 + random.random() / 10]]]))

				# Show detections on RGB and depth images.
				image = self.people_detector.draw_detections(image, people_bboxes, people_scores, people_ids, people_distances)
				image = self.face_detector.draw_detections(image, faces)
				cv2.imshow("Image", image)
				cv2.waitKey(1)

				if robot_stream:
					self.image_provider.release_image()

		except KeyboardInterrupt:
			print("Script interrupted by user, shutting down...")
			if send_data:
				self.data_sender.stop()
			if robot_stream:
				self.image_provider.disconnect()

		except Exception as e:
			 print("Thrown exception: " + str(e))
			 traceback.print_exc()
			 self.run()

if __name__ == "__main__":
	application = None

	if robot_stream:
		# Initialize robot's NAOqi framework.
		try:
			connection_url = "tcp://" + ip + ":" + str(port)
			application = qi.Application(["Main", "--qi-url=" + connection_url])
		except RuntimeError:
			print("Can't connect to NAOqi at \"" + ip + ":"  + str(port) +"\".\n"
				"Please check your script arguments. Run with -h option for help.")
			sys.exit(1)

	main = Main(application)
	main.run()

	#sys.path.remove(naoqi_path)