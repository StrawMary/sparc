import cfgs.config as cfg
import vision.vision_config as vision_cfg
import cv2
import numpy as np
import utils.yolo as yolo_utils
import utils.network as net_utils

from darknet import Darknet19

person_class_id = cfg.label_names.index('person')
h5_fname = 'models/yolo-voc.weights.h5'


class ObjectDetector:
	def __init__(self):
		print('Loading yolo model...')
		self.net = Darknet19()
		trained_model = cfg.trained_model
		net_utils.load_net(trained_model, self.net)
		self.net.cuda()
		self.net.eval()
		self.labels = cfg.label_names
		print('Loading model succeded.')

	def detect(self, image):
		im_data = np.expand_dims(
			yolo_utils.preprocess_test((image, None, cfg.inp_size))[0], 0)
		im_data = net_utils.np_to_variable(
			im_data, is_cuda=True, volatile=True).permute(0, 3, 1, 2)
		bbox_pred, iou_pred, prob_pred = self.net(im_data)
		bbox_pred = bbox_pred.data.cpu().numpy()
		iou_pred = iou_pred.data.cpu().numpy()
		prob_pred = prob_pred.data.cpu().numpy()

		bboxes, scores, classes = yolo_utils.postprocess(
			bbox_pred, iou_pred, prob_pred, image.shape, cfg, vision_cfg.yolo_people_detection_threshold)

		people = [(bboxes[i], scores[i]) for i in range(len(classes)) if classes[i] == person_class_id]

		objects = [(bboxes[i], scores[i], self.labels[classes[i]])
						for i in range(len(classes))
						if classes[i] != person_class_id and
							scores[i] >= vision_cfg.yolo_object_detection_threshold]

		return people, objects

	def draw_people_detections(self, image, people, people_ids, distances):
		bboxes = [np.append(people[i][0], people_ids[i]) for i in range(len(people))]
		scores = [people[i][1] for i in range(len(people))]
		class_ids = [person_class_id for i in range(len(people))]
		im2show = yolo_utils.draw_detections(image, bboxes, scores, class_ids, distances, cfg)

		if im2show.shape[0] > 1100:
			im2show = cv2.resize(
				im2show, (int(1000. * float(im2show.shape[1]) / im2show.shape[0]), 1000))
		return im2show

	def draw_object_detections(self, image, objects, distances):
		bboxes = [obj[0] for obj in objects]
		values = [obj[1] for obj in objects]
		class_ids = [obj[2] for obj in objects]
		im2show = yolo_utils.draw_detections(image, bboxes, values, class_ids, distances, cfg)

		if im2show.shape[0] > 1100:
			im2show = cv2.resize(
				im2show, (int(1000. * float(im2show.shape[1]) / im2show.shape[0]), 1000))
		return im2show
