import config as cfg
import os
import sys
folder_path = os.path.join(cfg.project_path, 'external_platform/vision/segmentation/')
sys.path.insert(0, os.path.join(folder_path, 'vision'))
sys.path.append(folder_path)

import numpy as np
import pytorch_segmentation_detection.models.resnet_dilated as resnet_dilated
import torch
from torchvision import transforms
from torch.autograd import Variable
from PIL import Image

class Segmentation():
    def __init__(self, fast_segmentation=True, width=640, height=480):
        print('Loading resnet model..')
        self.width = width
        self.height = height
        if fast_segmentation:
            self.fcn = resnet_dilated.Resnet18_8s(num_classes=21)
            self.fcn.load_state_dict(torch.load(os.path.join(folder_path, 'models/resnet_18_8s_59.pth')))
        else:
            self.fcn = resnet_dilated.Resnet34_8s(num_classes=21)
            self.fcn.load_state_dict(torch.load(os.path.join(folder_path, 'models/resnet_34_8s_68.pth')))
        self.fcn.cuda()
        self.fcn.eval()
        self.valid_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))
        ])
        print('Loading model succeded.')

    def segment(self, cv_image):
        image = Image.fromarray(cv_image).convert('RGB')
        image = Variable(self.valid_transform(image).unsqueeze(0).cuda())
        result = self.fcn(image)
        _, tmp = result.squeeze(0).max(0)
        segmentation = tmp.data.cpu().numpy().squeeze()
        mask = (segmentation == 15)
        segmented_image = np.uint8(mask * 255)
        return segmented_image, mask

    def segment_people(self, image, people):
        segmented_image = np.zeros((self.height, self.width))
        mask = np.zeros((self.height, self.width), dtype=bool)
        for person in people:
            bbox = person['person_bbox']
            if 'face_bbox' in person:
                bbox = person['face_bbox']

            left, top, right, bottom = bbox
            segmented_person, mask_person = self.segment(image[top:bottom, left:right])
            segmented_image[top:bottom, left:right] = segmented_person
            mask[top:bottom, left:right] = mask_person

        return segmented_image, mask

    def segment_people_bboxes(self, image, people):
        segmented_image = np.zeros((self.height, self.width))
        mask = np.zeros((self.height, self.width), dtype=bool)
        #return segmented_image, mask

        for bbox in people:
            left, top, right, bottom = bbox
            segmented_person, mask_person = self.segment(image[top:bottom, left:right])
            segmented_image[top:bottom, left:right] = segmented_person
            mask[top:bottom, left:right] = mask_person

        return segmented_image, mask

    def segment_bbox(self, image, bboxes):
        segmented_image = np.zeros((self.height, self.width))
        mask = np.zeros((self.height, self.width), dtype=bool)
        for bbox in bboxes:
            left, top, right, bottom = bbox
            segmented_person, mask_person = self.segment(
                image[top:bottom, left:right])
            segmented_image[top:bottom, left:right] = segmented_person
            mask[top:bottom, left:right] = mask_person

        return segmented_image, mask