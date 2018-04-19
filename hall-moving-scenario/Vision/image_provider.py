import numpy
import cv2
import time
import vision_definitions

from naoqi import ALProxy
from PIL import Image
import qi

class ImageProvider:

    def __init__(self, IP, PORT, FPS):
        self.IP = IP
        self.PORT = PORT
        self.FPS = FPS
        self.connected = False
        self.images_counter = 0

    def get_images_counter(self):
        return self.images_counter

    def reset(self):
        self.images_counter = 0

    def connect(self):
        self.cam_proxy = ALProxy("ALVideoDevice", self.IP, self.PORT)
        self.motion = ALProxy("ALMotion", self.IP, self.PORT)

        resolution = 2  # VGA
        color_space = 11 # RGB
        self.video_client = self.cam_proxy.subscribeCamera("camera_rgb", 0, resolution, color_space, self.FPS)

        color_space_depth = 0 # Depth
        resolution_depth = 1 # kQVGA
        self.video_client_depth = self.cam_proxy.subscribeCamera("camera_depth", 2, resolution_depth,
                                                     color_space_depth, 20)
        self.connected = True

    def get_cv_image(self):
        if(not self.connected):
            raise Exception('ALProxy not initialized.!')

        nao_image = self.cam_proxy.getImageRemote(self.video_client)
        nao_image_depth = self.cam_proxy.getImageRemote(self.video_client_depth)
        camInTorsoFrame = self.motion.getPosition("CameraTop", 1 , True)
        camInRobotFrame = self.motion.getPosition("CameraTop", 2 , True)

        self.images_counter = self.images_counter + 1
        image_width = nao_image[0]
        image_height = nao_image[1]
        array = nao_image[6]

        # Create a PIL Image from our pixel array.
        im = Image.frombytes("RGB", (image_width, image_height), array)
        opencv_image = cv2.cvtColor(numpy.array(im), cv2.COLOR_RGB2BGR)

        image_width_depth = nao_image_depth[0]
        image_height_depth = nao_image_depth[1]
        array_depth = nao_image_depth[6]

        # Create a PIL Image from our pixel array.
        im_depth = Image.frombytes("L", (image_width_depth, image_height_depth), array_depth)
        opencv_image_depth = numpy.array(im_depth)
        opencv_image_depth = cv2.resize(opencv_image_depth, (640,480))

        return opencv_image, opencv_image_depth, camInTorsoFrame, camInRobotFrame, qi.clockNow()

    def release_image(self):
        self.cam_proxy.releaseImage(self.video_client)
        self.cam_proxy.releaseImage(self.video_client_depth)

    def disconnect(self):
        self.cam_proxy.unsubscribe(self.video_client)
        self.cam_proxy.unsubscribe(self.video_client_depth)
        self.connected = False
