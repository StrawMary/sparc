import config as cfg
import json
import urllib2


class RemindersManager:
    def __init__(self, app, say_method):
        self.tabletService = app.session.service("ALTabletService")
        self.say = say_method
        self.signalID1 = None
        self.signalID2 = None
        self.on_fail = None
        self.on_success = None

    def get_reminders_count(self, target, on_success=None, on_fail=None):
        parsed_json = json.loads(urllib2.urlopen(cfg.REMINDERS_COUNT_URL).read())
        return int(parsed_json["count"])

    def display_reminders(self, target, on_success=None, on_fail=None):
        self.on_success = on_success
        self.on_fail = on_fail
        if cfg.robot_stream:
            if self.tabletService:
                self.signalID1 = self.tabletService.onJSEvent.connect(self.get_reminder)
                self.signalID2 = self.tabletService.onPageFinished.connect(self.page_finished)
                self.tabletService.showWebview(cfg.REMINDERS_URL)
            else:
                self.on_fail()
                self.clear_attrs()


    def get_reminder(self, reminder_text):
        self.say(reminder_text)

    def page_finished(self):
        script = """
            var reminder = document.getElementById("reminderValue").innerText;
            ALTabletBinding.raiseEvent(reminder);
        """
        self.tabletService.executeJS(script)

    def get_target_id_for_person(self, target):
        return 0

    def clear_display(self):
        self.tabletService.onPageFinished.disconnect(self.signalID1)
        self.tabletService.onJSEvent.disconnect(self.signalID2)
        if self.tabletService:
            self.tabletService.hideWebview()
            self.clear_attrs()

    def clear_attrs(self):
        self.on_success = None
        self.on_fail = None