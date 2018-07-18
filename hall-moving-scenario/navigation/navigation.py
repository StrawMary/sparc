import config as cfg
import matplotlib.pyplot as plt
import rospy
import tf
import tf2_ros
import sys

from actionlib_msgs.msg import *
from copy import deepcopy
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import distance
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from task_management.task_manager import ClassType, TaskType
from robot_localization import PepperLocalization


class Navigation:
	def __init__(self):
		if cfg.robot_stream:
			rospy.init_node('talker', anonymous=True)

		self.pepper_localization = PepperLocalization()

		if cfg.robot_stream:
			self.marker_array_publisher = rospy.Publisher('/detections', MarkerArray, queue_size=100)
			self.move_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

		self.map_pose = None
		self.object_ID = 0
		self.encountered_positions = {}
		self.possible_goals = []

		# self.fig = plt.figure()
		# self.ax = self.fig.add_subplot(111, projection='3d')


	def add_new_possible_goal(self, goal):
		self.possible_goals.append(goal)
		for i in range(len(self.possible_goals)):
			print(str(i+1) + ': ' + str(self.possible_goals[i]))


	def move_to_coordinate(self, goal):
		print('Move to: ' + str(goal))
		if cfg.robot_stream:
			self.move_publisher.publish(goal)


	def move_to_goal(self, key):
		goal = self.possible_goals[key]
		if goal in self.encountered_positions:
			self.move_to_coordinate(self.encountered_positions[goal][1])
			return True
		return False


	def move_to_map_pose(self):
		if self.map_pose:
			self.move_to_coordinate(self.map_pose)
			return True
		else:
			return False


	def publish_positions(self):
		markers = []
		i = 0
		for label in self.encountered_positions:
			marker = Marker(
				type=Marker.TEXT_VIEW_FACING,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=deepcopy(self.encountered_positions[label][1].pose),
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=self.encountered_positions[label][0],
				text=label)
			markers.append(marker)
			i += 1
			marker.pose.position.z += 0.3
			marker2 = Marker(
				type=Marker.SPHERE,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=self.encountered_positions[label][1].pose,
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=self.encountered_positions[label][0],
				text=label
			)
			markers.append(marker2)
			i += 1
		
		if cfg.robot_stream:
			self.marker_array_publisher.publish(markers)

		# for key in self.encountered_positions:
		# 	if task.class_type == ClassType.OBJECT or task.class_type == ClassType.QRCODE:
		# 		xs =[]
		# 		ys = []
		# 		zs = []
		# 		for pose in self.location_positions[key][1:]:
		# 			xs.append(pose.pose.position.x)
		# 			ys.append(pose.pose.position.y)
		# 			zs.append(pose.pose.position.z)
		# 			self.ax.scatter(xs, ys, zs)

		# 		self.ax.set_xlabel('X')
		# 		self.ax.set_ylabel('Y')
		# 		self.ax.set_zlabel('Z')
		# 		self.fig.savefig(key + '.png')
		# 		plt.cla()
		# 		print(key)
		# 		print(xs)
		# 		print(ys)
		# 		print(zs)


	def show_map_positions(self):
		self.publish_positions()


	def compute_euclidian_distance(self, p1, p2):
		a = (p1.x, p1.y, p1.z)
		b = (p2.x, p2.y, p2.z)
		return distance.euclidean(a, b)


	def add_tasks(self, tasks):
		self.run_tasks(tasks)


	def run_tasks(self, tasks):
		for task in tasks:
			if task.type == TaskType.SHOW_ON_MAP:
				self.run_task_show(task)
			elif task.type == TaskType.GO_TO:
				self.run_task_go_to(task)
			elif task.type == TaskType.FIND_OBJECT:
				self.run_task_find(task)


	def run_task_show(self, task):
		if cfg.robot_stream:
			robot_last_pose = self.pepper_localization.get_pose()
		else:
			robot_last_pose = PoseStamped()
		if not robot_last_pose:
			return

		loc = Pose()
		loc.position.x = task.coordinates[0]
		loc.position.y = task.coordinates[1]
		loc.position.z = task.coordinates[2]
		loc.orientation = robot_last_pose.pose.orientation
		pose = PoseStamped()
		pose.header.frame_id = 'laser'
		pose.pose = loc

		listener = tf.TransformListener()
		try:
			if cfg.robot_stream:
				listener.waitForTransform('/laser', '/map', rospy.Time(), rospy.Duration(4.0))
				pose_in_map = listener.transformPose('/odom', pose)
				pose_in_map.header.frame_id = 'odom'
			else:
				pose_in_map = PoseStamped()

			if task.class_type == ClassType.OBJECT:
				object_ref = task.label + str(self.object_ID)
				matched = None
				for key in self.encountered_positions:
					if key.startswith(task.label):
						if self.compute_euclidian_distance(
								self.encountered_positions[key][1].pose.position,
								pose_in_map.pose.position) < cfg.objects_distance_threshold:
							matched = key
				if not matched:
					self.encountered_positions[object_ref] = [ColorRGBA(0.5, 0.5, 0.0, 0.8), pose_in_map]
					self.object_ID += 1
					self.add_new_possible_goal(object_ref)
				else:
					self.encountered_positions[matched].append(pose_in_map)

			else:
				label = task.label
				if not task.label in self.encountered_positions:
					self.encountered_positions[task.label] = [ColorRGBA(0.5, 0.5, 1.0, 0.8), pose_in_map]
					if task.class_type == ClassType.PERSON:
						self.encountered_positions[task.label][0] = ColorRGBA(0.5, 1.0, 1.0, 0.8)
					self.add_new_possible_goal(task.label)
				else:
					self.encountered_positions[task.label][1] = pose_in_map

		except (tf2_ros.TransformException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass


	def run_task_go_to(self, task):
		pass

	def run_task_find(self, task):
		pass



