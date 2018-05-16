
from statistics import mode, StatisticsError
import pyqrcode
from pyzbar.pyzbar import decode
import pyttsx3
import cv2
from heapq import heappop, heappush
from collections import defaultdict
from statistics import mode

PERMANENT_USERS = set(['miruna', 'stefania', 'alex'])
TASK_TYPES = ['say_something', 'go_to_location', 'find_me_object']
TASK_NO = 1


import speech_recognition as sr

class Task:
    def __init__(self, id, type, person_name, message, location=None,
                 object=None, priority=10):
        self.taskID = id
        self.type = type
        self.priority = priority
        self.person_name = person_name
        self.message = message
        self.location = location
        self.object = object
        self.isDone = False

    def finishMessage(self):
        return self.message


    def isDone(self):
        return self.isDone

    def getStr(self):
        return "ID: %s Type: %s, Priority: %s, Person: %s, Message: %s" % \
               (self.taskID, self.type, self.priority, self.person_name, self.message)


class TaskManagement:
    def __init__(self):
        # priority queues for tasks
        self.say_something_tasks = [] # person
        self.go_to_tasks = [] # location
        self.find_me_smth = [] # object with having just an aproximate location
        self.taskNo = 0

        self.currentTask = None
        self.initTasks()


    def initTasks(self):
        self.addTask(Task(id=self.taskNo, type='say_something', person_name='miruna',
                          message='Have a nice day, Miruna', priority=2))
        self.addTask(Task(id=self.taskNo, type='say_something', person_name='stefania',
                          message='Have a nice day, Stefania', priority=3))
        self.addTask(Task(id=self.taskNo, type='go_to_location', person_name='alex',
                          message='Here is the bathroom, Alex',
                          location='bathroom', priority=3))

        self.getCurrentTask()


    def getDoableShortTask(self, peopleInView, locationInView, objectInView):
        doableTasks = []

        if peopleInView != []:
            for (_, _, task) in self.say_something_tasks:
                if task.person_name in peopleInView:
                    doableTasks.append(task)
        if locationInView != []:
            for (_, _, task) in self.go_to_tasks:
                if task.location in locationInView:
                    doableTasks.append(task)
        if objectInView != []:
            for (_, _, task) in self.find_me_smth:
                if task.object in objectInView:
                    doableTasks.append(task)

        for task in doableTasks:
            print("\t\tDoable task: %s" % (task.getStr()))

        return doableTasks


    def getCurrentTask(self):
        task = None
        if self.say_something_tasks != []:
            (_, _, task) = self.say_something_tasks[0]
        elif self.go_to_tasks != []:
            (_, _, task) = self.go_to_tasks[0]
        elif self.find_me_smth != []:
            (_, _, task) = self.find_me_smth[0]

        self.currentTask = task

        return self.currentTask


    def addTask(self, task):
        if task.type == 'say_something':
            heappush(self.say_something_tasks, (task.priority, task.taskID, task))
        elif task.type == 'go_to_location':
            heappush(self.go_to_tasks, (task.priority, task.taskID, task))
        elif task.type == 'find_me_object':
            heappush(self.find_me_smth, (task.priority, task.taskID, task))

        self.taskNo += 1

class TriggerGenerator:
    def __init__(self, results, shortTimeMemory):

        self.shortTimeMemory = shortTimeMemory

        # Dictionaries with known entities
        self.noConsecFaces = 60
        self.noConsecQRCodes = 10
        # name : [name or '' if it was seen in the last noConsecFaces frames]
        self.peopleFrames = defaultdict(list)
        # location : current_consecutive_frames
        self.qrCodeFrames = {'bathroom': [], 'lab_308': [], 'lab_303': [], 'lifts': []}

        # Dicionary with the unknown people
        # the IDs, the faces and the names of the last noConsecFacesToAdd the
        # biggest faces in the frames
        self.noConsecFacesToAdd = 30
        self.toAddPerson = defaultdict(list)#{'IDnames': [], 'faces': []}

        # variabila asta va fi false cand nu am de adaugat
        # o fac true cand a terminat de pus persoana aia in memorie
        self.addingToMemory = False
        self.personName = ''

        self.results = results

        self.speekEngine = pyttsx3.init()

        r = sr.Recognizer()
        m = sr.Microphone()
        with m as source:
            r.adjust_for_ambient_noise(source)
        stop_listening = r.listen_in_background(m, self.speechTrigger,
                                                phrase_time_limit=2.0)

    def addBiggestFrame(self, image, people, ids):

        if people:
            # gasesc ce mai mare fata
            biggestBoxIndex = -1
            biggestArea = -1
            for (box, _, _, _), index in zip(people, range(len(people))):
                left, top, right, bottom = box[0], box[1], box[2], box[3]
                # get the biggest bounding box with the id
                area = (right - left) * (bottom - top)
                if area > biggestArea:
                    biggestBoxIndex = index
                    biggestArea = area

            self.toAddPerson['IDname'].append((ids[biggestBoxIndex], people[biggestBoxIndex][1]))

            # adaug informatiile despre persoana aia
            bb = people[biggestBoxIndex][0]
            # cropez mai mult ca sa nu mai dea erori la alinierea fetei
            croppedFace = image[bb[1]-30:bb[3]+30, bb[0]-30:bb[2]+30, :]
            self.toAddPerson['faces'].append(croppedFace)
        else:
            self.toAddPerson['IDname'].append((-1, ''))
            self.toAddPerson['faces'].append([])

        # scot pe cele mai vechi
        self.toAddPerson['IDname'] = self.toAddPerson['IDname'][-self.noConsecFacesToAdd:]
        self.toAddPerson['faces'] = self.toAddPerson['faces'][-self.noConsecFacesToAdd:]

        # print(self.toAddPerson['IDname'])


    def addMemoryTrigger(self):
        if not self.addingToMemory:
            return

        # 1. Vreau sa retin persoana -> numele necunoscut
        # 2. Vreau sa retin persoana -> nume deja in self.personName
        try:
            (index, name) = mode(self.toAddPerson['IDname'])
            print('Numele persoanei de adaugat: %s' % (name))
            if self.personName == '' and name != '':
                if name not in self.shortTimeMemory.peopleEncountered.keys():
                    self.personName = name
                else:
                    self.addingToMemory = False
                    self.speekEngine.say('Hello again, %s' % (self.personName))
                    self.speekEngine.runAndWait()
                    return
        except Exception as e:
            (index, name) = (-1, '')


        if self.personName and index != -1:
            if len(self.toAddPerson['IDname']) < self.noConsecFacesToAdd - 1:
                return
            if self.personName in PERMANENT_USERS:
                self.speekEngine.say('Nice to see you today, %s' % (self.personName))
                self.speekEngine.runAndWait()
            else:
                self.speekEngine.say('Nice to meet you, %s' % (self.personName))
                self.speekEngine.runAndWait()

            # get relevant photos and save them
            some_faces = []
            for (indexPers, namePers), indexArray in zip(self.toAddPerson["IDname"],
                                    range(len(self.toAddPerson["IDname"]))):
                if indexPers == index:# and namePers == self.personName:
                    some_faces.append(self.toAddPerson['faces'][indexArray])

            # call the script to redo the classifier
            self.shortTimeMemory.addTemporaryPerson(some_faces, self.personName,
                                                    self.personName in PERMANENT_USERS)

            print("%s added in memory." % (self.personName))

            self.personName = ''
            self.addingToMemory = False


    def determinePresentEntity(self, dictionary, currentPresence, numberEntities):
        triggeredEntities = []

        for key in dictionary.keys():
            if key not in currentPresence:
                dictionary[key].append('none')

            dictionary[key] = dictionary[key][-numberEntities:]

            if len(dictionary[key]) == numberEntities:
                try:
                    locat = mode(dictionary[key])
                except StatisticsError:
                    continue

                if locat != 'none' and locat != '':
                    triggeredEntities.append(locat)

        return triggeredEntities


    def facesTrigger(self, people):
        # persoanele pe care le vad acum
        currentPeople = set([personName for (_, personName, _, _) in people])
        for personName in currentPeople:
            self.peopleFrames[personName].append(personName)

        triggeredEntities = self.determinePresentEntity(self.peopleFrames,
                                                        currentPeople,
                                                        self.noConsecFaces)
        if triggeredEntities != []:
            print('\tPeople near me: %s' % (triggeredEntities))
            self.results['people'] = triggeredEntities


    def qrCodesTrigger(self, frame):
        currentLocations = set()
        decodedMessage = decode(frame)
        if decodedMessage != []:
            location = str(decodedMessage[0].data.decode("utf-8"))
            currentLocations.add(location)
            self.qrCodeFrames[location].append(location)

        triggeredEntities = self.determinePresentEntity(self.qrCodeFrames,
                                                        currentLocations,
                                                        self.noConsecQRCodes)
        if triggeredEntities != []:
            print('\tLocations near me: %s' % (triggeredEntities))
            self.results['locations'] = triggeredEntities


    def speechTrigger(self, recognizer, audio):
        try:
            #print("You said " + recognizer.recognize_google(audio))
            self.results['speechRecog'] = recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            pass
            #print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            pass
            #print("Could not request results from service; {0}".format(e))


    def generateQRCodes(self):
        bathroom = pyqrcode.create("bathroom")
        bathroom.png("./qr_codes/bathroom.png", scale=20)

        lab_308 = pyqrcode.create("lab_308")
        lab_308.png("./qr_codes/lab_308.png", scale=20)

        lab_303 = pyqrcode.create("lab_303")
        lab_303.png("./qr_codes/lab_303.png", scale=20)

        lifts = pyqrcode.create("lifts")
        lifts.png("./qr_codes/lifts.png", scale=20)



'''
# (id, list of photoes with that face
self.toAddPerson = {'ID': -100, 'faces': [], 'name': ''}

def addMemoryTrigger(self, image, people, ids):

    if self.toAddPerson['ID'] == -100: # ma hotarasc pe o persoana
        biggestBoxIndex = -1
        biggestArea = -1
        for (box, _, _), index in zip(people, range(len(people))):
            left, top, right, bottom = box[0], box[1], box[2], box[3]
            # get the biggest bounding box with the id
            area = (right - left) * (bottom - top)
            if area > biggestArea:
                biggestBoxIndex = index
                biggestArea = area
        # put the id in pozitia de ID, daca persoana nu e in stm
        if people[biggestBoxIndex][2] == False:
            self.toAddPerson['ID'] = ids[biggestBoxIndex]
            self.toAddPerson['name'] = people[biggestBoxIndex][1]
            print("Pun persoana %s in short term memory." % (self.toAddPerson['name']))

    # caut ID-ul in ids
    try:
        faceIndex = ids.index(self.toAddPerson['ID'])
        if people[faceIndex][2]:
            raise Exception

        bb = people[faceIndex][0]
        croppedFace = image[bb[1]:bb[3], bb[0]:bb[2], :]
        self.toAddPerson['faces'].append(croppedFace)
    except Exception as e:
        print("Thrown exception: " + str(e))
        self.toAddPerson["ID"] = -100
        self.toAddPerson["faces"] = []
        self.toAddPerson['name'] = ''

    # daca am noConsecFacesToAdd => salvez pozele in folderul bun si ridic triggerul
    if len(self.toAddPerson['faces']) == self.noConsecFacesToAdd:
        for photo, index in zip(self.toAddPerson["faces"], range(self.noConsecFacesToAdd)):
            cv2.imwrite('./testFaces/person%s.png' % (index), photo)

        self.toAddPerson['ID'] = -100
        self.toAddPerson['faces'] = []
        name = self.toAddPerson['name']
        self.toAddPerson['name'] = ''

        return True, name

    return False, ''
'''