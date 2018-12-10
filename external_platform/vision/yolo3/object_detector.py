import config as cfg
import os
import traceback

from utils import *
from darknet import Darknet

folder_path = os.path.join(cfg.project_path, 'external_platform/vision/yolo3/')


class ObjectDetector:
    def __init__(self, use_cuda=True, classes=80, conf_threshold=0.5, nms_threshold=0.4):
        print('Loading yolo model...')
        self.network = Darknet(os.path.join(folder_path, 'cfg/yolov3.cfg'))
        self.network.load_weights(os.path.join(folder_path, 'models/yolov3.weights'))
        self.use_cuda = use_cuda
        if self.use_cuda:
            self.network.cuda()
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold

        if classes == 20:
            namesfile = os.path.join(folder_path, 'cfg/voc.names')
        elif classes == 80:
            namesfile = os.path.join(folder_path, 'cfg/coco.names')
        else:
            namesfile = os.path.join(folder_path, 'cfg/names')

        with open(namesfile, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]
        print('Loading model succeded.')

    def detect(self, image):
        resized_image = cv2.resize(image, (self.network.width, self.network.height))
        bboxes = detect(self.network, resized_image, self.conf_threshold, self.nms_threshold, self.use_cuda)
        height, width, _ = image.shape
        return process_detections(bboxes, width, height, self.labels)

    def draw_detections(self, image, detections):
        for (class_id, x, y, z, t, score) in detections:
            label = str(self.labels[class_id]) + ': ' + str(score)
            color = self.labels[class_id]
            cv2.rectangle(image, (x,y), (z,t), color, 2)
            cv2.putText(image, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return image

    def draw_people_detections(self, image, people, people_ids, distances):
        bboxes = [np.append(people[i][0], people_ids[i]) for i in range(len(people))]
        scores = [people[i][1] for i in range(len(people))]
        class_ids = [0 for i in range(len(people))]
        im2show = draw_detections(image, bboxes, scores, class_ids, distances, self.labels)

        if im2show.shape[0] > 1100:
            im2show = cv2.resize(
                im2show, (int(1000. * float(im2show.shape[1]) / im2show.shape[0]), 1000))
        return im2show

    def draw_object_detections(self, image, objects, distances):
        bboxes = [obj[0] for obj in objects]
        values = [obj[1] for obj in objects]
        class_ids = [obj[2] for obj in objects]
        im2show = draw_detections(image, bboxes, values, class_ids, distances, self.labels)

        if im2show.shape[0] > 1100:
            im2show = cv2.resize(
                im2show, (int(1000. * float(im2show.shape[1]) / im2show.shape[0]), 1000))
        return im2show


class Main:
    def __init__(self,):
        self.camera = cv2.VideoCapture(-1)
        self.detector = ObjectDetector()
        self.running = False

    def run(self):
        self.running = True
        try:
            while self.running:
                _, image = self.camera.read()
                start_time = time.time()
                detections = self.detector.detect(image)
                detection_time = time.time()
                self.detector.draw_detections(image, detections)

                print("%.2f s" % (detection_time - start_time))

                cv2.imshow('Image', image)
                key = cv2.waitKey(1)
                if key == 27:  # Esc key to stop
                    cv2.destroyAllWindows()
                    self.running = False

        except Exception as e:
            print('Thrown exception: ' + str(e))
            traceback.print_exc()


if __name__ == '__main__':
    main = Main()
    main.run()