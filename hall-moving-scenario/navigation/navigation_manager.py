import config as cfg
import rospy
import tf
import tf2_ros

from actionlib_msgs.msg import *
from copy import deepcopy
from enum import Enum
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from scipy.spatial import distance
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from navigation.target import Target
from robot_localization import PepperLocalization


KNOWN_LABELS = {8: 'chair', 10: 'table', 15: 'plant', 17: 'sofa', 19: 'monitor'}


class ClassType(Enum):
	PERSON = 1
	OBJECT = 2
	QRCODE = 3


class NavigationManager:
	def __init__(self):
		self.pepper_localization = PepperLocalization()

		if cfg.robot_stream:
			self.marker_array_publisher = rospy.Publisher('/detections', MarkerArray, queue_size=100)
			self.move_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

		self.map_pose = None
		self.object_ID = 0
		self.encountered_positions = {}
		self.possible_goals = []

	def add_new_possible_goal(self, goal):
		self.possible_goals.append(goal)

	def move_to_goal(self, key):
		goal = self.possible_goals[key]
		if goal in self.encountered_positions:
			self.move_to_coordinate(self.encountered_positions[goal][1])
			return True
		return False

	def move_to_coordinate(self, goal):
		if cfg.robot_stream:
			self.move_publisher.publish(goal)

	def is_located(self, key):
		return key in self.encountered_positions

	def get_coordinate_for_label(self, key):
		if key in self.encountered_positions:
			return self.encountered_positions[key][1]
		else:
			return None

	def move_to_map_pose(self):
		if self.map_pose:
			self.move_to_coordinate(self.map_pose)
			return True
		else:
			return False

	def compute_euclidian_distance(self, p1, p2):
		a = (p1.x, p1.y, p1.z)
		b = (p2.x, p2.y, p2.z)
		return distance.euclidean(a, b)

	def show_positions(self, data, value):
		targets = []
		if value == 'people':
			for info in data:  # info = [id, position, name]
				if len(info) > 2:
					show_target = Target(
						class_type=ClassType.PERSON,
						label=info[2],
						coordinates=info[1]
					)
					targets.append(show_target)

		elif value == 'qrcodes':
			for (label, position) in data:  # info = [label, position]
				show_target = Target(
					class_type=ClassType.QRCODE,
					label=label,
					coordinates=position
				)
				targets.append(show_target)

		elif value == 'objects':
			for (class_id, position) in data:  # info = [class_id, position]
				if class_id in KNOWN_LABELS:
					show_target = Target(
						class_type=ClassType.OBJECT,
						label=str(KNOWN_LABELS[class_id]),
						coordinates=position
					)
					targets.append(show_target)

		self.show_targets(targets)

	def run_task_go_to(self, task):
		self.move_to_coordinate(task.value)

	def run_task_find(self, task):
		pass

	def show(self, people_3d_positions, objects_3d_positions, qrcodes_3d_positions):
		self.show_positions(people_3d_positions, 'people')
		self.show_positions(objects_3d_positions, 'objects')
		self.show_positions(qrcodes_3d_positions, 'qrcodes')

		if cfg.debug_mode:
			self.publish_positions()

	def publish_positions(self):
		markers = []
		i = 0
		print(self.encountered_positions.keys())
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

	def show_targets(self, targets):
		for target in targets:
			self.show_target(target)

	def show_target(self, target):
		if cfg.robot_stream:
			robot_last_pose = self.pepper_localization.get_pose()
		else:
			robot_last_pose = PoseStamped()
		if not robot_last_pose:
			return

		loc = Pose()
		loc.position.x = target.coordinates[0]
		loc.position.y = target.coordinates[1]
		loc.position.z = target.coordinates[2]
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

			if target.class_type == ClassType.OBJECT:
				object_ref = target.label + str(self.object_ID)
				matched = None
				for key in self.encountered_positions:
					if key.startswith(target.label):
						if self.compute_euclidian_distance(
								self.encountered_positions[key][1].pose.position,
								pose_in_map.pose.position) < cfg.objects_distance_threshold:
							matched = key
				if not matched:
					self.encountered_positions[object_ref] = [cfg.object_color, pose_in_map]
					self.object_ID += 1
					self.add_new_possible_goal(object_ref)
				else:
					self.encountered_positions[matched].append(pose_in_map)

			else:
				if not target.label in self.encountered_positions:
					self.encountered_positions[target.label] = [cfg.qrcode_color, pose_in_map]
					if target.class_type == ClassType.PERSON:
						self.encountered_positions[target.label][0] = cfg.person_color
					self.add_new_possible_goal(target.label)
				else:
					self.encountered_positions[target.label][1] = pose_in_map

		except (tf2_ros.TransformException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

	def shut_down(self):
		pass


