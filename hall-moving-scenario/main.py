import sys
naoqi_path = "/home/sparc-308/workspace/stefania/pynaoqi-python2.7/lib/python2.7/site-packages"
sys.path.insert(0, naoqi_path)

import qi
import pickle
import socket
import time
import random
import sys
import cv2
import traceback
import tensorflow as tf
import numpy as np
import pyttsx

# Vision
# from Vision.Yolo2.people_detector import PeopleDetector
from Vision.FaceNet.face_det_rcog import FaceDetectRecog
from Vision.FaceNet.facenet import load_model
from Vision.FaceNet.detect_face import create_mtcnn
from Vision.Tracking.tracker import Tracker
from Utils.testThreadPool import ThreadPool
from Vision.ImageProvider.image_provider import ImageProvider
from PepperInteraction.reminders import RemindersChecker
from Vision.Segmentation.human_segmentation import HumanSegmentation
from TCPDataSender.data_sender import TCPDataSender
from Vision.Yolo2.people_detector import PeopleDetector
from Utils.data_processor import DataProcessor
from Utils.distances_smoothening import KalmanSmoother

# Trigger Generator
from TaskManagement.tasksManagement import TriggerGenerator, TaskManagement, Task

# Short time memory
from ShortTermMemory.short_time_memory import ShortTermMemory

robot_stream = True
send_data = False
ip_fast = "192.168.0.115"
ip_local = "172.19.11.65"

ip = ip_fast
port = 9559
frameRate = 30

recognition_threshold = 0.5

class Main(object):
    def __init__(self, app, sess, pnet, rnet, onet):
        self.people_detector = PeopleDetector()
        self.data_sender = TCPDataSender()
        self.data_processor = DataProcessor()
        self.human_segmentation = HumanSegmentation()
        self.image_provider = ImageProvider(ip, port, frameRate)
        self.distances_smoothening = None

        self.positions_no = 0
        self.trajectories = {}

        self.results = {'people': [], 'locations': [], 'speechRecog': '', 'objects':[]}

        # Vision
        # self.people_detector = PeopleDetector()
        self.face_detect_recog = FaceDetectRecog(sess, pnet, rnet, onet)
        self.tracker = Tracker()

        # Short time memory
        self.shortTimeMemory = ShortTermMemory(self.face_detect_recog)

        # Triggers objects and manage tasks
        self.triggerGenerator = TriggerGenerator(self.results,
                                                 self.shortTimeMemory)
        self.tasksManagement = TaskManagement()

        self.speakEngine = pyttsx.init()

        super(Main, self).__init__()

        if robot_stream:
            app.start()
            self.image_provider.connect()
        else:
            self.camera = cv2.VideoCapture(1)  # Get images from camera

        self.encountered_people = {} #


    def run(self):
        print("Waiting for connection...")
        if send_data:
            self.data_sender.start()
        print("Client connected.")

        pool = ThreadPool(4)

        try:
            while True:
                if robot_stream:
                    image, depth_image, camera_height, head_yaw, head_pitch = self.image_provider.get_cv_image()
                else:
                    _, image = self.camera.read()
                    depth_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                    camera_height = 1.2
                    head_yaw = 0
                    head_pitch = 0

                # # Use YOLO to detect people in frames.
                # pool.add_task(self.people_detector.detect, image, results)
                people_bboxes, people_scores = self.people_detector.detect(image)
                self.data_processor.square_detections(people_bboxes)

                # Compute people IDs.
                people_ids = self.tracker.updateIds(people_bboxes, people_scores)

                # Use FaceNet to detect and recognize faces in RGB image.
                faces = self.face_detect_recog.classifyFacesFromFrame(image,
                                        self.shortTimeMemory.model_temporary,
                                        self.shortTimeMemory.model_permanent)
                print(faces)

                # Associate detected faces with detected people.
                people = self.data_processor.associate_faces_to_people(people_bboxes, faces)

                # Segment humans in detections.
                segmented_image, mask = self.human_segmentation.get_segmented_image(image, people)

                # Compute more information about detected people.
                people_angles = self.data_processor.compute_people_angles(people)
                people_distances, depth_bboxes = self.data_processor.compute_people_distances(depth_image, mask, people)
                

                if len(people_distances) > 0:
                    if self.distances_smoothening == None:
                        self.distances_smoothening = KalmanSmoother(people_distances[0])
                    else:
                        distance = self.distances_smoothening.get_distance_estimation(people_distances[0])
                        if distance:
                            people_distances[0] = distance
                        else:
                            people = []
                            people_ids = []
                            people_angles = []
                            people_distances = []

                # Put all the informatin together.
                people_info = self.data_processor.get_people_info(people, people_ids, people_angles, people_distances)

                # Get people 3D position relative to the robot.
                people_3d_positions = self.data_processor.get_people_3d_positions(people_info, head_yaw, head_pitch, camera_height)


                # pool.wait_completion()
                # (people_bboxes, people_scores) = results['bodies']

                # Compute people IDs.
                facesBBoxes = [box for (box, _, _, _) in faces]
                faces_ids = self.tracker.updateIds(facesBBoxes,
                                                    np.ones(len(facesBBoxes)))

                # triggers that work in paralel
                pool.add_task(self.triggerGenerator.facesTrigger, faces)
                pool.add_task(self.triggerGenerator.qrCodesTrigger, image)
                pool.add_task(self.triggerGenerator.addBiggestFrame, image, faces, faces_ids)

                # handle short time memory situations
                speechRec = self.results['speechRecog']
                self.results['speechRecog'] = ''
                if 'hello' in speechRec:
                    self.triggerGenerator.addingToMemory = True
                    # aici ar trebui sa se puna un nume in self.personName, daca se stie
                    self.triggerGenerator.addMemoryTrigger()

                    if self.triggerGenerator.personName == '' and self.triggerGenerator.addingToMemory:
                        self.speakEngine.say('What is your name?')
                        self.speakEngine.runAndWait()
                elif speechRec and self.triggerGenerator.addingToMemory \
                        and self.triggerGenerator.personName == '':
                    self.triggerGenerator.personName = speechRec
                else:
                    self.triggerGenerator.addMemoryTrigger()


                # Show people detections on RGB image.
                imageWithBoxes = self.face_detect_recog.drawBBoxFaces(image,
                                                                      faces,
                                                                      faces_ids)
                # image = self.people_detector.draw_detections(imageWithBoxes,
                #                                              people_bboxes,
                #                                              people_scores)
                
                #cv2.imshow("Image", image)
                #cv2.waitKey(1)

                for position in people_3d_positions:
                    self.tasksManagement.addTask(Task())
                    # print(position)


                # Send positions.
                tasks = self.tasksManagement.getDoableShortTask(
                    peopleInView=self.results['people'],
                    locationInView=self.results['locations'],
                    objectInView=self.results['objects'])

                if send_data:
                    self.data_sender.send_data(pickle.dumps(people_3d_positions))
                    #print(people_3d_positions)
                    #self.data_sender.send_data(pickle.dumps([[0, [1 + random.random() / 10, 1 + random.random() / 10, 1 + random.random() / 10]]]))

                # Show detections on RGB and depth images.
                image = self.people_detector.draw_detections(imageWithBoxes, people_bboxes, people_scores, people_ids, people_distances)
                cv2.imshow("Image", image)
                cv2.waitKey(1)

                if robot_stream:
                    self.image_provider.release_image()

                pool.wait_completion()


        except KeyboardInterrupt:
            print("Script interrupted by user, shutting down...")
            if send_data:
                self.data_sender.stop()
            if robot_stream:
                self.image_provider.disconnect()

        except Exception as e:
             print("Thrown exception: " + str(e))
             traceback.print_exc()
             self.run()


if __name__ == "__main__":
    application = None

    if robot_stream:
        # Initialize robot's NAOqi framework.
        try:
            connection_url = "tcp://" + ip + ":" + str(port)
            application = qi.Application(["Main", "--qi-url=" + connection_url])
        except RuntimeError:
            print("Can't connect to NAOqi at \"" + ip + ":"  + str(port) +"\".\n"
                "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)

    with tf.Graph().as_default():
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)
        sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=False))
        with sess.as_default():
            modeldir = './Vision/FaceNet/pre_model/20170511-185253.pb'
            load_model(modeldir)
            pnet, rnet, onet = create_mtcnn(sess, './Vision/FaceNet/d_npy')

            main = Main(application, sess, pnet, rnet, onet)
            main.run()



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