from predefined_possible_locations import *
from std_msgs.msg import ColorRGBA
from subject_presentations import *

# Receive or send data to the robot
robot_stream = True
send_data = False

# Show images and positions.
debug_mode = False
show_markers = False
display_images = True

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
x_maximum_view_angle = 29.29  # horizontal field of view
y_maximum_view_angle = 22.82  # vertical field of view

# Face-person intersection threshold for matching.
intersection_percentage_thresh = 0.9

# Percentage of image area for detected object to be considered at 0.4 meters.
close_detection_area_percentage = 1.0 / 3.0

# Detection/recognition thresholds.
faces_recognition_threshold = 0.5
yolo_people_detection_threshold = 0.5
yolo_object_detection_threshold = 0.7

# Label for unknown faces.
unknown_name = 'unknown'

# Minimum euclidian distance between two objects of same class to be different.
objects_distance_threshold = 1.0

# Map colors to display.
person_color = ColorRGBA(0.0, 0.5, 0.5, 0.8)
object_color = ColorRGBA(1.0, 1.0, 0.5, 0.8)
qrcode_color = ColorRGBA(1.0, 0.5, 0.5, 0.8)

# Catch phrase to start speech recognition.
speech_catch_phrase = 'hey pepper'

# Eyes fade duration.
fade_duration = 1.0

# Task priorities.
GO_TO_PRIOR = 2
FIND_PRIOR = 3
SAY_PRIOR = 1

# Wit.ai api params.
URL = 'https://api.wit.ai/message'
access_keys = {'en-EN':  'VAYDJDTZRU4644WDEK4Q6YVXLY47F7GC', 'ro-RO': 'AOWWWDRYJW6C3MODYRQKJY25YSCNBLFD'}

# Wit.ai intent-entity association.
GO_TO_INTENT = 'go to'
FIND_INTENT = 'find'
SAY_INTENT = 'say'
intent_entities = {GO_TO_INTENT: 'target',
				   FIND_INTENT: 'target',
				   SAY_INTENT: 'target'}

# Association between subjects and presentations.
presentations = Dict({'lab308': lab308_presentation,
					  'lab303': lab303_presentation,
					  'lab306': lab306_presentation,
					  'robot': robot_presentation,
					  'alex': alex_presentation,
					  'stephanie': stephanie_presentation,
					  'time': get_time_presentation,
					  'default': default_presentation})

# Predefined positions for 'find' intent.
possible_locations = {'stephanie': stephanie_possible_locations,
					  'alex': alex_possible_locations}
