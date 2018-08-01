import urllib2
import json

from naoqi import ALProxy
from threading import Timer

class RemindersChecker:
    def __init__(self, pepper_ip, pepper_port, url="http://192.168.0.132:3000/RemindersCount"):
        self.url = url
        self.ip = pepper_ip
        self.port = pepper_port

        self.behaviour_manager = ALProxy("ALBehaviorManager", self.ip, self.port)
        self.behaviour_name = "multiplepeopletracker-4591b5/behavior_1"
        self.stopped = False

    def stop(self):
        self.stopped = True
        if(self.timer):
            self.timer.stop()
        print("Stopping behaviour.")
        if self.behaviour_manager.isBehaviorRunning(self.behaviour_name):
            self.behaviour_manager.stopBehavior(self.behaviour_name)

    def start_timer(self, interval):
        self.timer = Timer(interval, self.check_reminders, ()).start()

    def check_reminders(self):
        urlStr = urllib2.urlopen(self.url).read()
        parsed_json = json.loads(urlStr)
        number_of_reminders = int(parsed_json["count"])
        if number_of_reminders > 0:
            print("Found " + str(number_of_reminders) + " new reminders.")
            self.start_behaviour()
        if not self.stopped:
            self.start_timer(600)

    def start_behaviour(self):
        if self.behaviour_manager.isBehaviorInstalled(self.behaviour_name):
            if not self.behaviour_manager.isBehaviorRunning(self.behaviour_name):
                print("Starting behaviour...")
                self.behaviour_manager.startBehavior(self.behaviour_name)
            else:
                print("Behaviour is already running!")
        else:
            print("Behaviour is not installed!")