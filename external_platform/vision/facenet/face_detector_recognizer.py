from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import config as cfg
import vision.vision_config as vision_cfg
import cv2
import numpy as np
import os
import pickle
import tensorflow as tf
from scipy import misc
from . import facenet
from . import detect_face

from keras.preprocessing.image import img_to_array
from keras.models import load_model

folder_path = os.path.join(cfg.project_path, 'external_platform/vision/facenet/')
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


class FaceNetDetector:
    def __init__(self, classifier_name="faces_classifier", confidence_threshold=vision_cfg.facenet_recognition_threshold):
        print('Loading facenet model...')
        self.face_cascade = cv2.CascadeClassifier(os.path.join(folder_path, 'classifier/face_cascade_default.xml'))

        self.emotion_classifier = load_model(
            '/home/amiro/workspace/sparc/external_platform/vision/facenet/pre_model/model_v6_106.hdf5')

        with tf.device("/gpu:0"):
            with tf.Graph().as_default():
                gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)
                self.sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=False))
                with self.sess.as_default():
                    self.pnet, self.rnet, self.onet = detect_face.create_mtcnn(self.sess, os.path.join(folder_path, 'd_npy'))

                    self.confidence_threshold = confidence_threshold
                    self.minsize = 20  # minimum size of face
                    self.threshold = [0.6, 0.7, 0.7]  # three steps's threshold
                    self.factor = 0.709  # scale factor
                    self.margin = 44
                    self.img_2Dsize = (182, 182)
                    self.net_2Dsize = (160, 160)
                    self.net_size = 160

                    self.people_names = os.listdir(os.path.join(folder_path, 'out_dir'))
                    self.people_names.sort()

                    modeldir = os.path.join(folder_path, 'pre_model/20170511-185253.pb')
                    facenet.load_model(modeldir)

                    self.images_placeholder = tf.get_default_graph().get_tensor_by_name('input:0')
                    self.embeddings = tf.get_default_graph().get_tensor_by_name('embeddings:0')
                    self.phase_train_placeholder = tf.get_default_graph().get_tensor_by_name('phase_train:0')
                    self.embedding_size = self.embeddings.get_shape()[1]

                    classifier_filename = os.path.join(folder_path, 'classifier/' + classifier_name + '.pkl')
                    classifier_filename_exp = os.path.expanduser(classifier_filename)
                    with open(classifier_filename_exp, 'rb') as infile:
                        (self.model, class_names) = pickle.load(infile)
                self.detect(cv2.imread(os.path.join(folder_path, 'input_dir/load_image.png')))
            print('Loaded model succeded.')

    def detect_face(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        # Convert to (left, top, right, bottom)
        final_faces = []
        for (x, y, w, h) in faces:
            final_faces.append((x,y,x+w,y+h, 1))

        return final_faces

    def detect(self, frame):
        result = []

        bboxes = self.detect_face(frame)

        if frame.ndim == 2:
            frame = facenet.to_rgb(frame)
        frame = frame[:, :, 0:3]

        for bbox in bboxes:
            emb_array = np.zeros((1, self.embedding_size))

            if bbox[0] <= 0 or bbox[1] <= 0 or bbox[2] >= len(frame[0]) or bbox[3] >= len(frame):
                continue

            cropped = frame[bbox[1]:bbox[3], bbox[0]:bbox[2], :]
            cropped = facenet.flip(cropped, False)
            scaled = misc.imresize(cropped, self.img_2Dsize, interp='bilinear')
            scaled = cv2.resize(scaled, self.net_2Dsize, interpolation=cv2.INTER_CUBIC)
            scaled = facenet.prewhiten(scaled)
            scaled_reshape = scaled.reshape(-1, self.net_size, self.net_size, 3)
            feed_dict = {self.images_placeholder: scaled_reshape, self.phase_train_placeholder: False}
            emb_array[0, :] = self.sess.run(self.embeddings, feed_dict=feed_dict)
            predictions = self.model.predict_proba(emb_array)

            class_index = np.argmax(predictions, axis=1)[0]
            confidence = predictions[np.arange(1), class_index][0]

            if confidence >= self.confidence_threshold:
                result.append(([bbox[0], bbox[1], bbox[2], bbox[3]], self.people_names[class_index], confidence))
            else:
                result.append(([bbox[0], bbox[1], bbox[2], bbox[3]], vision_cfg.unknown_name, confidence))

        return result

    def draw_detections(self, image, faces):
        for (x, y, z, t), name, score in faces:
            if name == vision_cfg.unknown_name:
                color = vision_cfg.colors[vision_cfg.unknown_name]
            else:
                color = vision_cfg.colors['face']
            thick = 2
            cv2.rectangle(image, (x, y), (z, t), color, thick)
            label = '%s: %.3f' % (name, score)
            cv2.putText(image, label, (x, y - 12), 0, 0.6, color, 1)

        return image

    def draw_emotions(self, image, faces):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        emotion_dict = {'Angry' : 0, 'Sad' : 3, 'Neutral': 2, 'Happy': 1}
        class_labels = dict((v, k) for k, v in emotion_dict.items())
        for (x, y, z, t), name, score in faces:
            roi_gray = gray[y:t, x:z]
            try:
                roi_gray = cv2.resize(roi_gray, (48, 48), interpolation = cv2.INTER_AREA)
                roi = roi_gray.astype("float")/255.0
                roi = img_to_array(roi)
                roi = np.expand_dims(roi, axis = 0)

                predictions = self.emotion_classifier.predict(roi)[0]
                label = class_labels[predictions.argmax()]
                label_position = (x, y + 2)
                cv2.putText(image, label, label_position, 0, 0.6, (150, 150, 150), 2)
                #print(label)
            except:
                pass

        return image
