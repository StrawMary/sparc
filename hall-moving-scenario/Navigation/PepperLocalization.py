
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
import time
import sys
#import cv2
import tf
import numpy as np

class PepperLocalization:

    def __init__(self):
        rospy.Subscriber("/slam_out_pose", PoseStamped, self.update_pose)
        rospy.Subscriber("/map", OccupancyGrid, self.update_map)
        self.map = None
        self.latest_pose = None
        self.latest_map = None
        self.latest_map_pose = None
        self.listener = tf.TransformListener()

    def get_pose(self):
        return self.latest_pose

    def update_pose(self, data):
        self.latest_pose = data

    def update_map(self, data):
        self.map = data
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.resolution = data.info.resolution

        if(self.latest_pose):
            x = int(self.map_width / 2 + self.latest_pose.pose.position.x / self.resolution)
            y = int(self.map_height / 2 + self.latest_pose.pose.position.y / self.resolution)
            self.latest_map_pose = (x, y)
        data.data = [50 if x==-1 else x for x in data.data]
        self.latest_map = np.array([255 - i*2.55 for i in data.data]).reshape(data.info.width, data.info.height)
