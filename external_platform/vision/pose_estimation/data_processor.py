import vision.vision_config as cfg
import math


class DataProcessor:
    def __init__(self):
        self.bbox_thresh = cfg.close_detection_area_percentage * (cfg.width*cfg.height)
        self.cX = cfg.width/2
        self.cY = cfg.height/2

    ###############################################################################################
    #                     Distances relatively to the robot
    ###############################################################################################

    def get_point_distance(self, center, depth_image):
        x, y = center
        distance = depth_image[max(0, y)][min(639, x)]
        distance = float(distance) / 255.0 * 3.28 + 0.4
        return distance

    ###############################################################################################
    #                     Angles relatively to the center of the camera
    ###############################################################################################

    def compute_point_angles(self, center):
        x, y = center

        angle_x = ((self.cX - x) / float(self.cX)) * cfg.x_maximum_view_angle
        angle_y = ((y - self.cY) / float(self.cY)) * cfg.y_maximum_view_angle

        return math.radians(angle_x), math.radians(angle_y)

    ###############################################################################################
    #                     3D positions relatively to the robot
    ###############################################################################################

    def get_3d_position(self, distance, angles, camera_height):
        # Angle in radians on y direction in the robot's 3D coordinates (x on camera).
        angleY = angles[0]
        angleZ = angles[1]

        x = distance * math.cos(angleY)
        y = distance * math.sin(angleY)
        z = distance * math.sin(angleZ)
        position = (x, y, -z + camera_height)

        return position

    def get_point_3d_position(self, center, distance, camera_height):
        return self.get_3d_position(distance, self.compute_point_angles(center), camera_height)