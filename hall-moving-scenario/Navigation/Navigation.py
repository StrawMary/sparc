from PepperLocalization import PepperLocalization
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import tf
import tf2_ros
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Navigation:
	def __init__(self):
		self.person_positions = {}
		self.location_positions = {}
		self.map_pose = None
		rospy.init_node('talker', anonymous=True)
		self.pepper_localization = PepperLocalization()
		self.people_marke_array_publisher = rospy.Publisher('/people', MarkerArray, queue_size=100)
		self.locations_marker_array_publisher = rospy.Publisher('/qrCodes', MarkerArray, queue_size=100)
		self.move_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

		self.all_object_locations = {}
		self.possible_goals = []
		self.objectId = 0
		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(111, projection='3d')

	def add_new_possible_goal(self, goal):
		self.possible_goals.append(goal)
		for i in range(len(self.possible_goals)):
			print(str(i) + ":" + str(self.possible_goals[i]))

	def move_to_coordinate(self, goal):
		self.move_publisher.publish(goal)

	def move_to_goal(self, goal):
		if goal in self.person_positions:
			self.move_to_coordinate(self.person_positions[goal][0])
			return True
		elif goal in self.location_positions:
			self.move_to_coordinate(self.location_positions[goal][0])
			return True
		return False

	def move_to_map_pose(self):
		if self.map_pose:
			print("moving")
			self.move_to_coordinate(self.map_pose)
		else:
			return False

	def publish_persons(self):
		markers = []
		i = 0
		for person_name in self.person_positions:
			from copy import deepcopy
			marker = Marker(
				type=Marker.TEXT_VIEW_FACING,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=deepcopy(self.person_positions[person_name][0].pose),
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
				text=person_name)

			marker.pose.position.z += 0.3
			markers.append(marker)
			i += 1
			marker2 = Marker(
				type=Marker.SPHERE,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=self.person_positions[person_name][0].pose,
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
				text=str(person_name))
			markers.append(marker2)
			i += 1
		self.people_marke_array_publisher.publish(markers)

	def publish_locations(self):
		markers = []
		i = 0
		for location in self.location_positions:
			from copy import deepcopy
			marker = Marker(
				type=Marker.TEXT_VIEW_FACING,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=deepcopy(self.location_positions[location][0].pose),
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=ColorRGBA(0.0, 0.0, 1.0, 0.8),
				text=location)

			marker.pose.position.z += 0.3
			markers.append(marker)
			i += 1
			marker2 = Marker(
				type=Marker.SPHERE,
				id=i,
				lifetime=rospy.Duration(2.0),
				pose=self.location_positions[location][0].pose,
				scale=Vector3(0.2, 0.2, 0.2),
				header=Header(frame_id='odom'),
				color=ColorRGBA(0.0, 0.0, 1.0, 0.8),
				text=str(location))
			markers.append(marker2)
			i += 1
		self.locations_marker_array_publisher.publish(markers)

		for key in self.location_positions:
			xs =[]
			ys = []
			zs = []
			for pose in self.location_positions[key]:
				xs.append(pose.pose.position.x)
				ys.append(pose.pose.position.y)
				zs.append(pose.pose.position.z)
				self.ax.scatter(xs, ys, zs)

			self.ax.set_xlabel('X')
			self.ax.set_ylabel('Y')
			self.ax.set_zlabel('Z')
			self.fig.savefig(key + '.png')
			plt.cla()
			print(key)
			print(xs)
			print(ys)
			print(zs)

	def add_tasks(self, tasks):
		self.run_tasks(tasks)

	def refresh(self):
		self.publish_persons()
		self.publish_locations()

	def compute_euclidian_distance(self, p1, p2):
		from scipy.spatial import distance
		a = (p1.x, p1.y, p1.z)
		b = (p2.x, p2.y, p2.z)
		return distance.euclidean(a, b)

	def run_tasks(self, tasks):
		if not tasks or len(tasks) == 0:
			print("No valid tasks")
			return

		for task in tasks:
			if task.type == 'show_on_map':
				pose = PoseStamped()
				loc = Pose()
				last_pose = self.pepper_localization.get_pose()
				if not last_pose:
					return
				robot_pos = last_pose.pose
				loc.position.x = task.coordinates[0]
				loc.position.y = task.coordinates[1]
				loc.position.z = task.coordinates[2]
				loc.orientation = robot_pos.orientation
				pose.header.frame_id = 'laser'
				pose.pose = loc

				listener = tf.TransformListener()
				try:
					listener.waitForTransform("/laser", "/map", rospy.Time(), rospy.Duration(4.0))
					pose_in_map = listener.transformPose('/odom', pose)
					pose_in_map.header.frame_id = 'odom'

					if task.person_name:
						person_name = task.person_name
						if not person_name in self.person_positions:
							self.person_positions[person_name] = [pose_in_map]
							self.add_new_possible_goal(person_name)
							print("Found " + str(person_name))
						else:
							self.person_positions[person_name][0] = pose_in_map

					if task.location:
						location = task.location
						if not location in self.location_positions:
							self.location_positions[location] = [pose_in_map]
							self.add_new_possible_goal(location)
							print("Found " + str(location))
						else:
							self.location_positions[location][0] = pose_in_map

					if task.object:
						object = task.object
						refObj = str(object) + str(self.objectId)
						matched = None
						for key in self.location_positions:
							if key.startswith(object):
								if self.compute_euclidian_distance(self.location_positions[key][0].pose.position, pose_in_map.pose.position) < 1:
									matched = key
						if not matched:
							self.location_positions[refObj] = [pose_in_map]
							self.add_new_possible_goal(refObj)
							self.objectId += 1
						else:
							self.location_positions[matched].append(pose_in_map)






				except (tf2_ros.TransformException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
					pass
