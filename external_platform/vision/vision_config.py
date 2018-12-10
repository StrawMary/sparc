v2 = "v2"
v3 = "v3"

# YOLO version for object detection.
object_detection_version = v3

# External cameras.
external_cameras = {
	1: "rtsp://admin:admin@192.168.0.127/play1.sdp",
}

# Detection/recognition thresholds for YOLO and Facenet.
facenet_recognition_threshold = 0.5
yolo_people_detection_threshold = 0.5
yolo_object_detection_threshold = 0.7

# Label for unknown faces.
unknown_name = "unknown"

# QR code label.
qrcode_label = "ORCODE"

# Colors to draw bounding boxes.
colors = {
	"person": 		(127, 127, 0),
	"object": 		(127, 254, 254),
	qrcode_label: 	(127, 127, 254),
	"face": 		(0, 255, 0),
	unknown_name: 	(0, 0, 255),
}

text_thickness = 2

# Behaviour for moving the head to look around.
move_head = "movehead001/behavior_1"

# Camera fields of view information.
width = 640
height = 480
x_maximum_view_angle = 29.29  # horizontal field of view
y_maximum_view_angle = 22.82  # vertical field of view

# Top camera height.
top_camera_height = 1.165

# Face-person intersection threshold for matching.
intersection_percentage_thresh = 0.9

# Percentage of image area for detected object to be considered at 0.4 meters.
close_detection_area_percentage = 1.0 / 3.0

