from predefined_possible_locations import *
from std_msgs.msg import ColorRGBA
from subject_presentations import *
from utils.utils import *
# Receive or send data to the robot
robot_stream = False
receive_commands = True
send_data = False

# Show images and positions.
debug_mode = False
show_markers = True
display_images = True

# Robot IP information.
ip_fast = '192.168.0.115'
ip_local = '172.19.11.65'

# Image stream information.
ip = ip_fast
port = 9559
frameRate = 30

# Project path
project_path = '/home/sparc-308/workspace/sparc/'

# NaoQI path
naoqi_path = '/home/sparc-308/workspace/stefania/pynaoqi-python2.7/lib/python2.7/site-packages'

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
colors = {'person_color': ColorRGBA(0.0, 0.5, 0.5, 1.0),
		  'object_color': ColorRGBA(1.0, 1.0, 0.5, 1.0),
		  'qrcode_color': ColorRGBA(1.0, 0.5, 0.5, 1.0),
		  'person_default_color': ColorRGBA(0.2, 0.6, 1.0, 1.0)}

# Catch phrase to start speech recognition.
speech_catch_phrase = 'hey pepper'

# Eyes fade duration.
fade_duration = 1.0

# Task priorities.
SAY_PRIOR = 1
SEARCH_PRIOR = 2
GO_TO_PRIOR = 3
FIND_PRIOR = 4
SHOW_REMINDERS_PRIOR = 1
HEALTH_PRIOR = 1

# Wit.ai api params.
URL = 'https://api.wit.ai/message'
access_keys = {'en-EN':  'VAYDJDTZRU4644WDEK4Q6YVXLY47F7GC', 'ro-RO': 'AOWWWDRYJW6C3MODYRQKJY25YSCNBLFD'}

# Wit.ai intent-entity association.
SAY_INTENT = 'say'
SEARCH_INTENT = 'look'
GO_TO_INTENT = 'go to'
FIND_INTENT = 'find'
STOP_INTENT = 'stop'
HELLO_INTENT = 'hello'
REMINDERS_INTENT = 'reminders'
NEXT_INTENT = 'next'
PREVIOUS_INTENT = 'previous'
HEALTH_INTENT = 'health'
hello_response = 'hello'

mandatory_intent_entities = {SAY_INTENT: ['target'],
							 SEARCH_INTENT: ['target'],
							 GO_TO_INTENT: ['target'],
							 FIND_INTENT: ['target'],
							 REMINDERS_INTENT: ['target'],
							 NEXT_INTENT: [],
							 PREVIOUS_INTENT: [],
							 HEALTH_INTENT: ['health_entity'],
							 STOP_INTENT: [],
							 HELLO_INTENT: []}

# Association between subjects and presentations.
presentations = {'lab308': lab308_presentation,
				'lab303': lab303_presentation,
				'lab306': lab306_presentation,
				'home': robot_presentation,
				'alex': alex_presentation,
				'stephanie': stephanie_presentation,
				'time': get_time_presentation,
				'weather': get_weather_report,
				'default': default_presentation}

last_known_positions_file = 'positions.p'

default_positions = {
	'alex': [5.7, 6.2, 1],
	'stephanie': [6.0, 6.7, 1],
	'home': [5.5, 10.0, 0]
}

HEALTH_MEASUREMENTS_URL = {
	'heart': "http://192.168.0.158:3000/HeartRate",
	'blood_pressure': "http://192.168.0.158:3000/BloodPressure",
	'steps': "http://192.168.0.158:3000/Steps",
	'weight': "http://192.168.0.158:3000/Weight",
	'sleep': "http://192.168.0.158:3000/Sleep"
}

TIME_SHOWING_HEALTH_MEASUREMENTS = 5

KNOWN_LABELS = {8: 'chair', 10: 'table', 15: 'plant', 17: 'sofa', 19: 'monitor'}

REMINDERS_COUNT_URL = "http://192.168.0.158:3000/remindersCount"
REMINDERS_URL = "http://192.168.0.158:3000/Reminders/0"