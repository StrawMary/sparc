
from statistics import mode, StatisticsError
import pyqrcode
from pyzbar.pyzbar import decode

class Task:
    def __init__(self, type, person_id=None, message=None, location=None,
                 object=None):
        # self.taskID = TASK_NO + 1
        # TASK_NO += 1
        self.type = type
        self.person_id = person_id
        self.message = message
        self.location = location
        self.object = object

    def action(self):
        # coenzile date lui pepper
        # se poate reapela si fac planningul de la punctul respectiv
        pass

    def isDone(self):
        # depinde daca stiu persoana sau nu
        pass

class TriggerGenerator:
    def __init__(self):
        self.noConsecFaces = 60
        self.noConsecQRCodes = 10

        self.peopleFrames = {}  # name : [name or '' if it was seen in the last noConsecFaces frames]
        self.qrCodeFrames = {'bathroom': [], 'lab_308': [], 'lab_303': [],
                             'lifts': []}  # location : current_consecutive_frames

    def facesTrigger(self, people):

        currentPeople = set([personName for (_, personName, _) in people])
        triggeredPeople = []

        for personName in currentPeople:
            if personName in self.peopleFrames:
                l = self.peopleFrames[personName]
                l.append(personName)
                self.peopleFrames[personName] = l
            else:
                self.peopleFrames[personName] = [personName]

        for key in self.peopleFrames.keys():
            l = self.peopleFrames[key]

            if key not in currentPeople:
                l.append('none')

            l = l[-self.noConsecFaces:]
            if len(l) == self.noConsecFaces:
                try:
                    pers = mode(l)
                except StatisticsError:
                    continue

                if pers != 'none':
                    triggeredPeople.append(pers)

            self.peopleFrames[key] = l

        return triggeredPeople

    def qrCodesTrigger(self, frame):

        triggeredLocations = []
        currentLocations = []

        decodedMessage = decode(frame)
        if decodedMessage != []:
            location = str(decodedMessage[0].data.decode("utf-8"))
            currentLocations.append(location)
            print(location)

            l = self.qrCodeFrames[location]
            l.append(location)
            self.qrCodeFrames[location] = l

        for key in self.qrCodeFrames.keys():
            l = self.qrCodeFrames[key]

            if key not in currentLocations:
                l.append('none')

            l = l[-self.noConsecQRCodes:]
            if len(l) == self.noConsecQRCodes:
                try:
                    locat = mode(l)
                except StatisticsError:
                    continue

                if locat != 'none':
                    triggeredLocations.append(locat)

            self.qrCodeFrames[key] = l

        return triggeredLocations

        # decodedMessage = decode(frame)
        # if decodedMessage != []:
        #     location = str(decodedMessage[0].data.decode("utf-8"))
        #     print(location)
        #     self.qrCodeFrames[location] = min(self.noConsecQRCodes, self.qrCodeFrames[location] + 1)
        #
        #     if self.qrCodeFrames[location] >= self.noConsecQRCodes:
        #         return location
        #
        #     for key in self.qrCodeFrames.keys():
        #         if key != location:
        #             self.qrCodeFrames[key] = max(0, self.qrCodeFrames[key] - 1)
        # else:
        #     for key in self.qrCodeFrames.keys():
        #         self.qrCodeFrames[key] = max(0, self.qrCodeFrames[key] - 1)
        #
        # return ''

    def generateQRCodes(self):
        bathroom = pyqrcode.create("bathroom")
        bathroom.png("./qr_codes/bathroom.png", scale=20)

        lab_308 = pyqrcode.create("lab_308")
        lab_308.png("./qr_codes/lab_308.png", scale=20)

        lab_303 = pyqrcode.create("lab_303")
        lab_303.png("./qr_codes/lab_303.png", scale=20)

        lifts = pyqrcode.create("lifts")
        lifts.png("./qr_codes/lifts.png", scale=20)