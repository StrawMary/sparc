import config as cfg
import cv2
import math
import numpy as np


class DataProcessor:
    def __init__(self):
        self.bbox_thresh = cfg.close_detection_area_percentage * (cfg.width*cfg.height)
        self.cX = cfg.width/2
        self.cY = cfg.height/2

    ###############################################################################################
    #                     Association between faces and people detections
    ###############################################################################################

    def square_detections(self, bboxes_and_scores):
        for i in range(len(bboxes_and_scores)):
            width = bboxes_and_scores[i][0][2] - bboxes_and_scores[i][0][0]
            bboxes_and_scores[i][0][3] = bboxes_and_scores[i][0][1] + width

    def get_intersection_percentage(self, person_bbox, face_bbox):
        p_left, p_top, p_right, p_bottom = person_bbox
        f_left, f_top, f_right, f_bottom = face_bbox

        left = max(p_left, f_left)
        right = min(p_right, f_right)
        top = max(p_top, f_top)
        bottom = min(p_bottom, f_bottom)

        area = (f_right - f_left) * (f_bottom - f_top)
        intersection_area = (right - left) * (bottom - top)

        return intersection_area * 1.0 / area

    def associate_faces_to_people(self, people, faces):
        people_data = []
        for i in range(len(people)):
            # Checks if a face is detected for the detected person.
            face_detected = False
            for face in faces:
                intersection_percentage = self.get_intersection_percentage(people[i][0], face[0])
                if intersection_percentage >= cfg.intersection_percentage_thresh:
                    face_detected = True
                    break

            # Adds person information.
            person = {'person_bbox': people[i][0], 'person_score': people[i][1]}
            if face_detected:
                person['face_bbox'] = face[0]
                person['name'] = face[1]
                person['confidence'] = face[2]
            people_data.append(person)

        return people_data

    ###############################################################################################
    #                     Distances relatively to the robot
    ###############################################################################################

    def compute_distance(self, depth_image, bbox, real_bbox=[], segmented_pixels=[]):
        # Adjust position from RGB image to depth image.
        left, top, right, bottom = bbox
        translated_bbox = [min(639, left + 5), max(0, top - 10), min(639, right + 5), max(0, bottom - 10)]
        left, top, right, bottom = translated_bbox

        # Compute distance based on depth image.
        detection = depth_image[top:bottom, left:right]
        if segmented_pixels != []:
            detection = segmented_pixels
        distance = np.median(detection)
        distance = distance / 255 * 3.28 + 0.4
        if distance == 0.4:
            if real_bbox != []:
                left, top, right, bottom = real_bbox
            if (right - left) * (bottom - top) < cfg.close_detection_area_percentage * cfg.width * cfg.height:
                distance = 5.0

        return distance, translated_bbox

    def compute_objects_distances(self, depth_image, objects):
        distances = []
        depth_bboxes = []

        for object_bbox, _, _ in objects:
            distance, depth_bbox = self.compute_distance(depth_image, object_bbox)
            distances.append(distance)
            depth_bboxes.append(depth_bbox)

        return distances, depth_bboxes

    def compute_people_distances(self, depth_image, mask_image, people):
        distances = []
        depth_bboxes = []

        segmented_image = depth_image * mask_image

        for person in people:
            bbox = person['person_bbox']
            if 'face_bbox' in person:
                bbox = person['face_bbox']

            # Compute distance based on depth image.
            left, top, right, bottom = bbox
            detection = segmented_image[top:bottom, left:right]
            segmented_pixels = detection[np.nonzero(detection)]
            if len(segmented_pixels) == 0:
                distance, depth_bbox = self.compute_distance(depth_image, bbox, person['person_bbox'])
            else:
                distance, depth_bbox = self.compute_distance(segmented_image, bbox, person['person_bbox'], segmented_pixels)

            distances.append(distance)
            depth_bboxes.append(depth_bbox)

        return distances, depth_bboxes

    ###############################################################################################
    #                     Angles relatively to the center of the camera
    ###############################################################################################

    def get_center(self, left, top, right, bottom):
        cX = left + (right - left) / 2
        cY = top + (bottom - top) / 2
        return [cX, cY]

    def compute_angles(self, bbox):
        left, top, right, bottom = bbox
        x, y = self.get_center(left, top, right, bottom)

        angleX = ((self.cX - x) / float(self.cX)) * cfg.x_maximum_view_angle
        angleY = ((top - self.cY) / float(self.cY)) * cfg.y_maximum_view_angle
        
        return (math.radians(angleX), math.radians(angleY))

    def compute_objects_angles(self, objects):
        angles = []
        for bbox, _, _ in objects:
            angles.append(self.compute_angles(bbox))
        return angles

    def compute_people_angles(self, people):
        angles = []

        for person in people:
            bbox = person['person_bbox']
            #if 'face_bbox' in person:
            #    bbox = person['face_bbox']
            
            angles.append(self.compute_angles(bbox))
        return angles

    ###############################################################################################
    #                     3D positions relatively to the robot
    ###############################################################################################

    def get_3d_position(self, distance, angles, head_yaw, head_pitch, camera_height):
        # Angle in radians on y direction in the robot's 3D coordinates (x on camera).
        angleY = angles[0] + head_yaw
        angleZ = angles[1] + head_pitch

        x = distance * math.cos(angleY)
        y = distance * math.sin(angleY)
        z = distance * math.sin(angleZ)
        position = (x, y, -z + camera_height)

        return position

    def get_objects_3d_positions(self, objects, distances, head_yaw, head_pitch, camera_height):
        positions = []
        object_angles = self.compute_objects_angles(objects)

        for i, (_, value, class_id) in enumerate(objects):
            if distances[i] < 5.0:
                position = self.get_3d_position(
                    distances[i],
                    object_angles[i],
                    head_yaw,
                    head_pitch,
                    camera_height
                )
                if class_id == 'QRCODE':
                    positions.append((value.lower(), position))
                else:
                    positions.append((class_id, position))

        return positions

    def get_people_3d_positions(self, people, people_ids, people_distances, head_yaw, head_pitch, camera_height):
        people_angles = self.compute_people_angles(people)

        positions = []
        for i, person in enumerate(people):
            if people_distances[i] < 5.0:
                position = self.get_3d_position(
                    people_distances[i],
                    people_angles[i],
                    head_yaw,
                    head_pitch,
                    camera_height
                )
                if 'name' in person:
                    positions.append((people_ids[i], position, person['name'].lower()))
                else:
                    positions.append((people_ids[i], position))

        return positions

    ###############################################################################################

    def draw_squares(self, depth_image, centers):
        depth_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
        for x, y, z, t in centers:
            cv2.rectangle(depth_image, (int(x), int(y)), (int(z), int(t)), (0, 255, 0), 2)
        return depth_image