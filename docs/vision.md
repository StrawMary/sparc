# Vision configuration

The vision component is composed of a set of independent modules, each responsible for one particular vision task (person detection, face recognition, QR code recognition, etc.). The implementation is located in the **sparc/external_platform/vision** folder, with the modules distributed in the following folder hierarchy :

-   **facenet** - contains [FfaceNnet](https://arxiv.org/abs/1503.03832) network for face recognition and [Haar ](https://docs.opencv.org/3.3.0/d7/d8b/tutorial_py_face_detection.html)cascades for face detection
-   **image_provider** - contains the script that gets the stream of images from the robot
-   **qrcodes_handler** - contains the script that generates, detects and recognizes QR codes
-   **segmentation** - contains [Resnet-18-8s](https://github.com/warmspringwinds/pytorch-segmentation-detection) network for person segmentation
-   **tracking** - contains implementation of [SORT](https://arxiv.org/abs/1602.00763) algorithm for object tracking
-   **yolo2** - contains [YOLOv2](https://pjreddie.com/darknet/yolo/) network for object detection
- **data_processor.py** - auxiliary file for processing data that comes from the vision modules
- **vision_manager.py** - file for combining all the modules' outputs into a single vision result
- **run_vision.py** - main file for running the vision component as a stand-alone application

## Software Prerequisites

To test the vision component both as a stand-alone application or within the big project, the machine must install all the following required packages:
 - Cython (version >=0.27.3)
 - H5py (version >= 2.7.1)
 - Matplotlib
 - Naoqi (version >=2.5)
 - NumPy
 - OpenCV-Python (version >=3.3.0.10)
 - Python (version 2.7)
 - Pillow
 - Psutil
 - PyCrayon (version >=0.5)
 - PyTorch
 - PyZBar
 - Requests
 - TensorFlow (version >=1.2)
 - Scikit-image
 - Scikit-learn
 - SciPy  

## Testing the vision component

    python run_vision.py [-r] [-v]

The script gets input images from a video camera or the robot's camera and displays 3 images:
 - **RGB image** - input image with overlapped detections
   - **Blue bbox** - detected person. The text above represents the person ID, accuracy of detection and distance to the person
   - **Yellow bbox** - detected object. The text above represents the object name, accuracy of detection and distance to the object
   - **Pink bbox** - detected QR code. The text above represents the QR code ID, accuracy of detection and distance to the QR code
   - **Green/Red bbox** - recognized/unrecognized face. The text above represents the person name and accuracy of recognition
  - **Depth image** - depth image with overlapped bboxes representing detected people
  - **Segmented image** - black and white image, where the white pixels represent the segmented detected person   tensorflow==1.2   Scipy   scikit-learn   matplotlib   Pillow  
   requests   Psutil
   
   Numpy
   
   Scikit-image
   
   pyzbar
   
   Naoqi 2.5.5 (if robot input required)

  

## Testing the vision component

    python run_vision.py [-r] [-v]

The script gets input images from a video camera or the robot's camera and displays 3 images:

 - List item

Optional arguments:
-r, --robot_stream = use the input from the robot's camera. By default the script uses the video camera of the machine.
-v, --verbose = display information about execution time.

## Fine-tuning the face recognition module

The input directory that contains the training images to fine-tune the pre-trained model are in **sparc/external_platform/vision/facenet/input_dir**.

The pre-trained model is located in **sparc/external_platform/vision/facenet/pre_model**, while the fine-tuned model is located in **sparc/external_platform/vision/facenet/classifier**.

1.  **Adding a new face to the database**
-   Add a new folder containing a set of images of the new face in **sparc/external_platform/vision/facenet/input_dir**. The folder name should match the person name.
-   Align the face dataset using python aligndata_first.py`
-   Retrain the last layer of the pre-trained model using `python create_classifier_se.py`

2.  **Removing existing faces**
-   Delete the folder associated with the face to be removed from **sparc/external_platform/vision/facenet/input_dir**
-   Retrain the last layer of the pre-trained model using `“python create_classifier_se.py`

## Generate new QR codes

The images representing the existing QR codes are located in **sparc/external_platform/vision/qrcodes_handler/qr_codes** folder.
To generate new QR codes:
1. Delete existing images from  **qrcodes_handler/qr_codes**
2. Modify the **generate_QRcodes** function in **qrcodes_handler/qrcodes_handler.py** and add each new QR code as follows:

    new_qr_code = pyqrcode.create('new_qr_code_name')
    new_qr_code.png('./qr_codes/new_qr_code_image.png', scale=20)

3. Run `python qrcodes_handler.py`

<!--stackedit_data:

eyJoaXN0b3J5IjpbLTIwODg3NDY2MTJdfQ==

-->