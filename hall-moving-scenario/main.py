import sys

naoqi_path = "/home/sparc-308/workspace/stefania/pynaoqi-python2.7/lib/python2.7/site-packages"
sys.path.insert(0, naoqi_path)

import cv2
import pickle
import socket
import traceback
import tensorflow as tf
from multiprocessing import Process, Queue
from multiprocessing.pool import ThreadPool

# Vision
from Vision.image_provider import ImageProvider
from Vision.Yolo2.people_detector import PeopleDetector
from Vision.FaceNet.face_det_rcog import FaceDetectRecog
from Vision.FaceNet.facenet import load_model
from Vision.FaceNet.detect_face import create_mtcnn

# TCP Communication
from TCPCommunication.data_sender import TCPDataSender

# Trigger Generator
from TaskManagement.tasksManagement import TriggerGenerator

# Short time memory
from ShortTermMemory.short_time_memory import ShortTermMemory

send_data = False
robot_stream = True
ip_fast = "192.168.0.115"
ip_local = "172.19.11.32"

ip = ip_fast
port = 9559
frameRate = 30

class Main(object):
    def __init__(self, app, sess, pnet, rnet, onet):
        self.data_sender = TCPDataSender()

        # Vision
        self.people_detector = PeopleDetector()
        self.face_detect_recog = FaceDetectRecog(sess, pnet, rnet, onet)
        self.triggerGenerator = TriggerGenerator()

        # Short time memory
        self.shortTimeMemory = ShortTermMemory(self.face_detect_recog)

        # recognition_threshold = 0.5
        # self.face_detect_recog = FaceNetDetector("./Vision/FaceNet", recognition_threshold)

        super(Main, self).__init__()
        if robot_stream:
            self.image_provider = ImageProvider(ip, port, frameRate)  # from Pepper
            app.start()
            self.image_provider.connect()
        else:
            self.camera = cv2.VideoCapture(-1)  # Get images from camera

    def run(self):
        print("Waiting for connection...")
        if send_data:
            self.data_sender.start()
        print("Client connected.")

        pool = ThreadPool(processes=1)

        try:
            index = 0
            while True:
                if robot_stream:
                    image, depth_image, torsoFrame, robotFrame, timestamp = self.image_provider.get_cv_image()

                    # cv2.imshow('image', image)
                    # k = cv2.waitKey(1)
                    # if k == 27:  # wait for ESC key to exit
                    #     cv2.destroyAllWindows()
                    # elif k == ord('s'):  # wait for 's' key to save and exit
                    #     cv2.imwrite('/home/sparc-308/workspace/miruna/new_scenario/poze_noi/stefania%s.png' % (index), image)
                    #     index += 1
                else:
                    _, image = self.camera.read() # Get images from camera

                    torsoFrame = []
                    robotFrame = []
                    timestamp = []

                # cv2.imshow("Image", image)
                # cv2.waitKey(1)

                # Without processes --------------------------------------------
                # Use YOLO to detect people in the RGB image.
                people_bboxes, people_scores = self.people_detector.detect(image)
                #
                # # Use FaceNet to detect and recognize faces in RGB image.
                people, imageWithBoxes = self.face_detect_recog.classifyFacesFromFrame(image,
                                        self.shortTimeMemory.model_temporary,
                                        self.shortTimeMemory.model_permanent)

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



                # Send data to robot.
                if send_data:
                    people_info = people_info[0:2]

                    toSend = [people_info, torsoFrame, robotFrame, timestamp]
                    self.data_sender.send_data(pickle.dumps(toSend))

                # Show people detections on RGB image that already has faces boxes.
                image = self.people_detector.draw_detections(imageWithBoxes,
                                                             people_bboxes,
                                                             people_scores)
                cv2.imshow("Image", image)
                cv2.waitKey(1)

                if robot_stream:
                    self.image_provider.release_image()


        except KeyboardInterrupt:
            print("Script interrupted by user, shutting down...")
            if send_data:
                self.data_sender.stop()
            if robot_stream:
                self.image_provider.disconnect()
            sys.exit(0)

        except socket.error as error_message:
            print("Socket error: " + str(
                error_message) + "\nDisconnected... Reconnecting ...")
            if send_data:
                self.data_sender.stop()
            self.run()

        except Exception as e:
            print("Thrown exception: " + str(e))
            if send_data:
                self.data_sender.stop()
            if robot_stream:
                self.image_provider.disconnect()
            traceback.print_exc()
            sys.exit(0)


if __name__ == "__main__":
    application = None

    import qi

    if robot_stream:
        # Initialize robot's NAOqi framework.
        try:
            connection_url = "tcp://" + ip + ":" + str(port)
            application = qi.Application(["Main", "--qi-url=" + connection_url])
        except RuntimeError:
            print(
            "Can't connect to NAOqi at \"" + ip + ":" + str(port) + "\".\n"
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

    sys.path.remove(naoqi_path)



'''
# With a threadpool --------------------------------------------
async_result1 = pool.apply_async(self.people_detector.detect, (image))
async_result2 = pool.apply_async(self.face_detect_recog.classifyFacesFromFrame,
                                 (image, self.shortTimeMemory.model_temporary,
                                    self.shortTimeMemory.model_permanent))

people_bboxes, people_scores = async_result1.get()
people, imageWithBoxes = async_result2.get()
'''