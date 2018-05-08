import sys
import cv2
import traceback
import tensorflow as tf
import numpy as np
import pyttsx3

# Vision
# from Vision.Yolo2.people_detector import PeopleDetector
from Vision.FaceNet.face_det_rcog import FaceDetectRecog
from Vision.FaceNet.facenet import load_model
from Vision.FaceNet.detect_face import create_mtcnn
from Vision.Tracking.tracker import Tracker
from Utils.testThreadPool import ThreadPool

# Trigger Generator
from TaskManagement.tasksManagement import TriggerGenerator

# Short time memory
from ShortTermMemory.short_time_memory import ShortTermMemory

class Main(object):
    def __init__(self, sess, pnet, rnet, onet):

        self.results = {'bodies': (), 'faces': [], 'peopleMet': [],
                        'locations': [], 'speechRecog': ''}

        # Vision
        # self.people_detector = PeopleDetector()
        self.face_detect_recog = FaceDetectRecog(sess, pnet, rnet, onet)
        self.tracker = Tracker()

        # Short time memory
        self.shortTimeMemory = ShortTermMemory(self.face_detect_recog)

        # Triggers objects
        self.triggerGenerator = TriggerGenerator(self.results,
                                                 self.shortTimeMemory)
        self.speekEngine = pyttsx3.init()

        super(Main, self).__init__()
        self.camera = cv2.VideoCapture(1)  # Get images from camera


    def run(self):

        pool = ThreadPool(4)

        while True:

            _, image = self.camera.read() # Get images from camera

            # cv2.imshow('image', image)
            # k = cv2.waitKey(1)
            # if k == 27:  # wait for ESC key to exit
            #     cv2.destroyAllWindows()
            # elif k == ord('s'):  # wait for 's' key to save and exit
            #     cv2.imwrite('/home/sparc-308/workspace/miruna/new_scenario/poze_noi/stefania%s.png' % (index), image)
            #     index += 1

            # print(index)
            # index += 1

            # # Use YOLO to detect people in frames.
            # pool.add_task(self.people_detector.detect, image, results)

            # Use FaceNet to detect and recognize faces in RGB image.
            people = self.face_detect_recog.classifyFacesFromFrame(image,
                                    self.shortTimeMemory.model_temporary,
                                    self.shortTimeMemory.model_permanent)

            # pool.wait_completion()
            # (people_bboxes, people_scores) = results['bodies']

            # Compute people IDs.
            facesBBoxes = [box for (box, _, _) in people]
            people_ids = self.tracker.updateIds(facesBBoxes,
                                                np.ones(len(facesBBoxes)))

            # triggers that work in paralel
            pool.add_task(self.triggerGenerator.facesTrigger, people)
            pool.add_task(self.triggerGenerator.qrCodesTrigger, image)
            pool.add_task(self.triggerGenerator.addBiggestFrame, image, people, people_ids)

            # handle short time memory situations
            speechRec = self.results['speechRecog']
            self.results['speechRecog'] = ''
            if 'hello' in speechRec:
                self.triggerGenerator.addingToMemory = True
                # aici ar trebui sa se puna un nume in self.personName, daca se stie
                self.triggerGenerator.addMemoryTrigger()

                if not self.triggerGenerator.personName and self.triggerGenerator.addingToMemory:
                    self.speekEngine.say('What is your name?')
                    self.speekEngine.runAndWait()
            elif speechRec and self.triggerGenerator.addingToMemory \
                    and self.triggerGenerator.personName == '':
                self.triggerGenerator.personName = speechRec
            else:
                self.triggerGenerator.addMemoryTrigger()


            # Show people detections on RGB image.
            imageWithBoxes = self.face_detect_recog.drawBBoxFaces(image,
                                                                  people,
                                                                  people_ids)
            # image = self.people_detector.draw_detections(imageWithBoxes,
            #                                              people_bboxes,
            #                                              people_scores)
            cv2.imshow("Image", image)
            cv2.waitKey(1)

            pool.wait_completion()



if __name__ == "__main__":

    with tf.Graph().as_default():
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)
        sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=False))
        with sess.as_default():
            modeldir = './Vision/FaceNet/pre_model/20170511-185253.pb'
            load_model(modeldir)
            pnet, rnet, onet = create_mtcnn(sess, './Vision/FaceNet/d_npy')

            main = Main(sess, pnet, rnet, onet)

            try:
                main.run()
            except KeyboardInterrupt:
                print("Script interrupted by user, shutting down...")
                sys.exit(0)

            except Exception as e:
                print("Thrown exception: " + str(e))
                traceback.print_exc()
                sys.exit(0)



"""

#window = pyglet.window.Window()
keys = key.KeyStateHandler()
#window.push_handlers(keys)

# Get images for short termmemory update.
# Pe asta nu pot sa il pun pe thread separat deoarece:
# QObject::startTimer: Timers cannot be started from another
# thread nu am reusit nici cu alte metode de get key sa iau
# self.getImageForSTM(image, people, people_ids)
def getImageForSTM(self, image, people, people_ids):

    # k = cv2.waitKey(1)
    # if k == ord('c'):
    # Check if the spacebar is currently pressed:
    if keys[key.SPACE]:
        print("Am apasat space")
        self.captureFrame = True
    if self.captureFrame:
        finish, name = self.triggerGenerator.addMemoryTrigger(
            copy.deepcopy(image), people, people_ids)
        if finish:
            print("Done capturing faces: %s" % (name))
            self.captureFrame = False
"""


'''
# With a threadpool --------------------------------------------
async_result1 = pool.apply_async(self.people_detector.detect, (image))
async_result2 = pool.apply_async(self.face_detect_recog.classifyFacesFromFrame,
                                 (image, self.shortTimeMemory.model_temporary,
                                    self.shortTimeMemory.model_permanent))

people_bboxes, people_scores = async_result1.get()
people, imageWithBoxes = async_result2.get()
'''

# from multiprocessing import Process, Queue
# Start the processes for tasks --------------------------------
# Use YOLO to detect people in the RGB image.
# resultPeopleDetect = Queue()
# procPeopleDetect = Process(target=self.people_detector.detect,
#                            args=(image, resultPeopleDetect))
# procPeopleDetect.start()
#
# Use FaceNet to detect and recognize faces in RGB image.
# resultFaceRecog = Queue()
# procFaceRecog = Process(
#     target=self.face_detect_recog.classifyFacesFromFrame,
#     args=(image, self.shortTimeMemory.model_temporary,
#           self.shortTimeMemory.model_permanent,
#           resultFaceRecog))
# procFaceRecog.start()
#
# # Join the processes -------------------------------------------
# procPeopleDetect.join()
# (people_bboxes, people_scores) = resultPeopleDetect.get()
# procFaceRecog.join()
# (people, imageWithBoxes) = resultFaceRecog.get()