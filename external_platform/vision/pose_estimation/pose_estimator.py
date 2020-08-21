import cv2
import os
import sys
import vision.vision_config as cfg

sys.path.append('vision/pose_estimation')

from data_processor import DataProcessor
from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import get_graph_path
from tf_pose import common


class PoseEstimator:
	def __init__(self):
		print("Loading openpose model...")
		self.no_images_received = 0

		self.data_processor = DataProcessor()
		self.pose_estimator = TfPoseEstimator(get_graph_path('mobilenet_thin'), target_size=(432, 368))
		print("Loading model succeded.")

	def post_process_poses(self, humans, depth_image):
		poses = []
		display_poses = []
		for k, human in enumerate(humans):
			pose = [None for _ in range(common.CocoPart.Background.value)]
			display_pose = [None for _ in range(common.CocoPart.Background.value)]
			for i in range(common.CocoPart.Background.value):
				if i not in human.body_parts.keys():
					continue

				body_part = human.body_parts[i]
				center = (int(body_part.x * cfg.width + 0.5), int(body_part.y * cfg.height + 0.5))
				distance = self.data_processor.get_point_distance(center, depth_image)
				pose[i] = self.data_processor.get_point_3d_position(center, distance, camera_height=1.2)
				display_pose[i] = (center[0], center[1])

			poses.append((k, pose))
			display_poses.append((k, display_pose))
		return poses, display_poses

	def draw_poses(self, image, poses, x_translation=0, y_translation=0, depth=False):
		for id, pose in poses:
			for i in range(common.CocoPart.Background.value):
				if pose[i]:
					cv2.circle(
						image,
						(pose[i][0]+x_translation, pose[i][1]+y_translation),
						3,
						((255, 255, 255) if depth else common.CocoColors[i]),
						thickness=3,
						lineType=8,
						shift=0,
					)
			for pair_order, pair in enumerate(common.CocoPairsRender):
				if pose[pair[0]] and pose[pair[1]]:
					cv2.line(
						image,
						(pose[pair[0]][0]+x_translation, pose[pair[0]][1]+y_translation),
						(pose[pair[1]][0]+x_translation, pose[pair[1]][1]+y_translation),
						((255, 255, 255) if depth else common.CocoColors[pair_order]),
						3,
					)
		return image

	def estimate_poses(self, image, depth_image):
		if len(image) == 0:
			return [], []

		estimated_poses = self.pose_estimator.inference(image, resize_to_default=False, upsample_size=4.0)
		return self.post_process_poses(estimated_poses, depth_image)

	def check_hand_up(self, pose):
		if not pose:
			return False

		right_hand_up = True
		left_hand_up = True
		joints = pose[1]
		for i in range(common.CocoPart.Neck.value, common.CocoPart.LAnkle.value):
			if not joints[common.CocoPart.LWrist.value] or \
					(joints[i] and joints[i][1] < joints[common.CocoPart.LWrist.value][1]):
				left_hand_up = False
			if not joints[common.CocoPart.RWrist.value] or \
					(joints[i] and joints[i][1] < joints[common.CocoPart.RWrist.value][1]):
				right_hand_up = False

		return left_hand_up or right_hand_up

