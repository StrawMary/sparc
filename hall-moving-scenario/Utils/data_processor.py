import cv2
import math
import numpy as np

bbox_thresh = 1.0/3.0 * (640*480)
intersection_percentage_thresh = 0.9

cX = 320
cY = 240
#x_maximum_view_angle = 28.6
#y_maximum_view_angle = 22.15
x_maximum_view_angle = 29.29
y_maximum_view_angle = 22.82

class DataProcessor():
    def __init__(self):
        #self.orb_detector = cv2.ORB_create()
        self.last_id = 0


    def square_detections(self, bboxes):
        for i in range(len(bboxes)):
            width = bboxes[i][2] - bboxes[i][0]
            bboxes[i][3] = bboxes[i][1] + width


    def compute_people_ids(self, image2, bboxes2, prev_data):
        if not prev_data:
            ids = []
            for i in range(len(bboxes2)):
                ids.append(self.last_id)
                self.last_id = self.last_id + 1
            return ids

        image1, bboxes1, ids1 = prev_data
        ids2 = [-1 for i in range(len(bboxes2))]

        distances = []

        for i in range(len(bboxes1)):
            left1, top1, right1, bottom1 = bboxes1[i]
            if bottom1-top1 < 50 or right1-left1 < 50:
                continue
            crop1 = image1[top1:bottom1, left1:right1]
            kp1, des1 = self.orb_detector.detectAndCompute(crop1, None)

            for j in range(len(bboxes2)):
                left2, top2, right2, bottom2 = bboxes2[j]
                if bottom2 - top2 < 50 or right2 - left2 < 50:
                    continue
                crop2 = image2[top2:bottom2, left2:right2]
                kp2, des2 = self.orb_detector.detectAndCompute(crop2, None)
                if des2 is None:
                    continue
                bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
                matches = bf.match(des1, des2)
                distance = sum([x.distance for x in matches]) / (len(matches) + 1.0)

                distances.append((i, j, distance))

        distances = sorted(distances, key=lambda x: x[2])

        for (i, j, dist) in distances:
            if ids2[j] == -1 and ids1[i] not in ids2:
                ids2[j] = ids1[i]

        for j in range(len(bboxes2)):
            if ids2[j] == -1:
                ids2[j] = self.last_id
                self.last_id += 1

        return ids2


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


    def associate_faces_to_people(self, people_bboxes, faces):
        people = []
        for i in range(len(people_bboxes)):
            # Checks if a face is detected for the detected person.
            face_detected = False
            for face in faces:
                intersection_percentage = self.get_intersection_percentage(people_bboxes[i], face[0])
                if intersection_percentage >= intersection_percentage_thresh:
                    face_detected = True
                    break

            # Adds person information.
            person = {'person_bbox': people_bboxes[i]}
            if face_detected:
                person['face_bbox'] = face[0]
                person['name'] = face[1]
                person['confidence'] = 100.0
            people.append(person)

        return people


    def compute_people_distances(self, depth_image, mask_image, people):
        distances = []
        centers = []

        segmented_image = depth_image * mask_image

        for person in people:
            bbox = person['person_bbox']
            person_left, person_top, person_right, person_bottom = person['person_bbox']
            if 'face_bbox' in person:
                bbox = person['face_bbox']

            # Adjust position from RGB image to depth image.
            left, top, right, bottom = bbox
            bbox = [min(639, left + 5), max(0, top - 10), min(639, right + 5), max(0, bottom - 10)]

            # Get segmented pixels in depth image if it is the case.
            detection = segmented_image[top:bottom, left:right]
            segmented_pixels = detection[np.nonzero(detection)]
            if len(segmented_pixels) == 0:
                cropped = depth_image[top:bottom, left:right]
            else:
                cropped = segmented_pixels

            # Compute distance based on depth image.
            left, top, right, bottom = bbox
            distance = np.median(cropped)
            distance = distance / 255 * 3.28 + 0.4
            if distance == 0.4:
                if (person_right - person_left) * (person_bottom - person_top) < bbox_thresh:
                    distance = 5.0
            distances.append(distance)
            centers.append(bbox)

        return distances, centers


    def get_center(self, left, top, right, bottom):
        cX = left + (right - left) / 2
        cY = top + (bottom - top) / 2
        return [cX, cY]  


    def compute_people_angles(self, people):
        angles = []

        for person in people:
            bbox = person['person_bbox']
            #if 'face_bbox' in person:
            #    bbox = person['face_bbox']
            left, top, right, bottom = bbox

            x, y = self.get_center(left, top, right, bottom)

            angleX = ((cX - x) / float(cX)) * x_maximum_view_angle
            angleY = ((top - cY) / float(cY)) * y_maximum_view_angle

            angles.append((math.radians(angleX), math.radians(angleY)))
        return angles


    def create_person_info(self, bbox, person_id, distance, angles, name, confidence):
        left, top, right, bottom = bbox
        angleX, angleY = angles
        if confidence >= 0.0:
            return [person_id, left, top, right, bottom, distance, [angleX, angleY], name, confidence]
        return [person_id, left, top, right, bottom, distance, [angleX, angleY]]


    def get_people_info(self, people, people_ids, people_angles, people_distances):
        people_info = []
        for i in range(len(people)):
            if people_distances[i] == 5.0 or people_distances[i] == 0.4:
                continue
            bbox = people[i]['person_bbox']
            name = ''
            confidence = -1.0
            if 'face_bbox' in people[i]:
                bbox = people[i]['face_bbox']
                name = people[i]['name']
                confidence = people[i]['confidence']

            pid = people_ids[i]
            angles = people_angles[i]
            distance = people_distances[i]
            person_info = self.create_person_info(bbox, pid, distance, angles, name, confidence)
            people_info.append(person_info)

        return people_info


    def get_people_3d_positions(self, people_info, head_yaw, head_pitch, camera_height):
        positions = []
        for person in people_info:
            # Angle in radians on y direction in the robot's 3D coordinated (x on camera).
            angleY = person[6][0] + head_yaw
            angleZ = person[6][1] + head_pitch

            x = person[5] * math.cos(angleY)
            y = person[5] * math.sin(angleY)
            z = person[5] * math.sin(angleZ)
            position = (x, y, -z + camera_height)
            if len(person) > 7:
                positions.append((person[0], position, person[7]))
            else:
                positions.append((person[0], position))

        return positions


    def draw_squares(self, depth_image, centers):
        depth_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
        for x, y, z, t in centers:
            cv2.rectangle(depth_image, (int(x), int(y)), (int(z), int(t)), (0, 255, 0), 2)
        return depth_image