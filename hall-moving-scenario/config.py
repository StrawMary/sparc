from std_msgs.msg import ColorRGBA

# Receive or send data to the robot
robot_stream = True
send_data = False

# Robot IP information.
ip_fast = '192.168.0.115'
ip_local = '172.19.11.65'

# Image stream information.
ip = ip_fast
port = 9559
frameRate = 30

# Camera fields of view information.
width = 640
height = 480
x_maximum_view_angle = 29.29 # horizontal field of view
y_maximum_view_angle = 22.82 # vertical field of view

# Face-person intersection threshold for matching.
intersection_percentage_thresh = 0.9

# Percentage of image area for detected object to be considered at 0.4 meters.
close_detection_area_percentage = 1.0/3.0

# Detection/recognition thresholds.
faces_recognition_threshold = 0.5
yolo_people_detection_threshold = 0.5
yolo_object_detection_threshold = 0.7

# Label for unknown faces.
unknown_name = 'unknown'

# Minimum euclidian distance between two objects of same class to be different.
objects_distance_threshold = 1.0

# Show images and positions.
debug_mode = True

# Task priorities.
GO_TO_PRIOR = 3
FIND_PRIOR = 2
SAY_PRIOR = 1

# Map colors to display.
person_color = ColorRGBA(0.0, 0.5, 0.5, 0.8)
object_color = ColorRGBA(1.0, 1.0, 0.5, 0.8)
qrcode_color = ColorRGBA(1.0, 0.5, 0.5, 0.8)

# Catch phrase to start speech recognition.
speech_catch_phrase = 'hey pepper'

# Eyes fade duration.
fade_duration = 1.0
