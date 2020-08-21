import config as cfg
import cv2
import generate_detections
import numpy as np
import os
import tensorflow as tf
import sys
sys.path.append('vision/deep_tracking')

from deep_sort import nn_matching
from deep_sort.detection import Detection
from deep_sort.tracker import Tracker

folder_path = os.path.join(cfg.project_path, 'external_platform/vision/deep_tracking/')


class ObjectTracker:
    def __init__(self):
        cfg = dict({
            'allow_soft_placement': False,
            'log_device_placement': False
        })

        self.graph = tf.Graph()
        with tf.device("/gpu:0"):
            with self.graph.as_default() as g:
                utility = 0.5
                cfg['gpu_options'] = tf.GPUOptions(
                    per_process_gpu_memory_fraction = utility)
                cfg['allow_soft_placement'] = True

                self.sess = tf.Session(config = tf.ConfigProto(**cfg))
                self.sess.run(tf.global_variables_initializer())

        metric = nn_matching.NearestNeighborDistanceMetric("cosine", 0.2, 100)
        self.tracker = Tracker(metric)
        self.encoder = generate_detections.create_box_encoder(
            os.path.join(folder_path, "resources/networks/mars-small128.ckpt-68577"))

    def track_objects(self, image, people):
        bboxes = []
        scores = []
        identifiers = []
        for i, (x, y, z, t, score, person_id) in enumerate(people):
            bboxes.append(np.array([x, y, (z-x), (t-y)]).astype(np.float64))
            scores.append(score)
            identifiers.append(i)
        bboxes = np.array(bboxes)
        scores = np.array(scores)
        identifiers = np.array(identifiers)

        features = self.encoder(image, bboxes.copy())
        detections = [Detection(bbox, score, feature, i) 
                        for bbox, score, feature, i in 
                        zip(bboxes, scores, features, identifiers)]

        self.tracker.predict()
        self.tracker.update(detections)

        new_detections = []
        for track in self.tracker.tracks:
            if track.info in identifiers:
                print(track.info)
                people[track.info][5] = track.track_id
            bbox = track.to_tlbr()
            new_detections.append(list(bbox) + [track.confidence, track.track_id])

        return people

    def update_ids(self, image, people):
        bboxes = []
        scores = []
        identifiers = []
        for i, ((x, y, z, t), score) in enumerate(people):
            bboxes.append(np.array([x, y, (z-x), (t-y)]).astype(np.float64))
            scores.append(score)
            identifiers.append(i)
        bboxes = np.array(bboxes)
        scores = np.array(scores)
        identifiers = np.array(identifiers)

        features = self.encoder(image, bboxes.copy())
        detections = [Detection(bbox, score, feature, i) 
                        for bbox, score, feature, i in 
                        zip(bboxes, scores, features, identifiers)]

        self.tracker.predict()
        self.tracker.update(detections)

        ids = [-1 for person in people]
        for track in self.tracker.tracks:
            if track.info in identifiers:
                ids[track.info] = track.track_id
        return ids
        


if __name__ == "__main__":
    tracker = ObjectTracker()