#matplotlib inline

import sys, os
sys.path.insert(0, '../../../../vision/')
sys.path.append('../../../../../pytorch-segmentation-detection/')

# Use second GPU -pytorch-segmentation-detection- change if you want to use a first one
os.environ["CUDA_VISIBLE_DEVICES"] = '0'

from PIL import Image
from matplotlib import pyplot as plt

import torch
from torchvision import transforms
from torch.autograd import Variable
import pytorch_segmentation_detection.models.resnet_dilated as resnet_dilated

import numpy as np
import cv2

camera = cv2.VideoCapture(0)

img_path = 'demo_img_vittal.jpg'

valid_transform = transforms.Compose(
                [
                     transforms.ToTensor(),
                     transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
                ])

fcn = resnet_dilated.Resnet34_8s(num_classes=21)
fcn.load_state_dict(torch.load('resnet_34_8s_68.pth'))
fcn.cuda()
fcn.eval()

while True:
    _, image = camera.read()
    #cv2_im = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    pil_im = Image.fromarray(image)

    img_not_preprocessed = pil_im.convert('RGB')
    img = valid_transform(img_not_preprocessed)
    img = img.unsqueeze(0)
    img = Variable(img.cuda())

    res = fcn(img)

    _, tmp = res.squeeze(0).max(0)

    segmentation = tmp.data.cpu().numpy().squeeze()
    prediction_mask = (segmentation == 15)
    png_transparancy_mask = np.uint8(prediction_mask * 255)

    cv2.imshow("Image", image)
    cv2.imshow("Segmented", png_transparancy_mask)
    cv2.waitKey(1)
