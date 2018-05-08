# Short term memory class 

# Idei de incercat:
    # poate pe viitor doar updatez clasificatorul existent ... dupa ce vad ca mi-a iesit prima parte
    # poate pe viitor mut instantierea de clasificatori altundeva si ii reinitializez doar daca s-au adaugat persoane

    
# Cum am persoanele:
# Permanent: Miruna, Stefania, Alex Gavril
# Short term memory: Stefania, Alex Gavril
# Nicaieri: Alex Sorici
    
import copy
import cv2
import os
import pickle
import subprocess
from statistics import mode, StatisticsError
from TaskManagement.tasksManagement import Task

PERMANENT_USERS = ['miruna', 'stefania', 'alex']
pwd = "/home/miruna/Desktop/new_scenario_pepper/"

permanentAlignedPeopleFolder = "./ShortTermMemory/permanent_aligned_faces_dir"
temporaryAlignedPeopleFolder = "./ShortTermMemory/temporary_aligned_faces_dir"
newPersonAlignedFacesFolder = "./ShortTermMemory/aligned_dir"

permanentPeopleFolder = "./ShortTermMemory/permanent_faces_dir"
temporaryPeopleFolder = "./ShortTermMemory/temporary_faces_dir"
newPersonFacesFolder = "./ShortTermMemory/to_align_dir"

tasksFolder="./ShortTermMemory/tasks"

permanentPeopleClassifier = "./ShortTermMemory/classifiers/permanentClassifier.pkl"
temporaryPeopleClassifier = "./ShortTermMemory/classifiers/temporaryClassifier.pkl"
starterPermanentPeopleClassifier = "./ShortTermMemory/permanentClassifier.pkl"
starterTemporaryPeopleClassifier = "./ShortTermMemory/temporaryClassifier.pkl"


        
class Person:
    #person_number = 0
    
    def __init__(self, name):
        self.name = name
        #self.ID = person_number
        #person_number += 1
        self.tasks = []
        
        # fac un fisier pickle cu taskurile persoanei aleia
        pickle.dump(self.tasks, open("%s/%s.pkl" % (tasksFolder, str(self.name)), "wb" ))

    def addTask(self, taskType, message=None, location=None, object=None):
        newTask = Task(taskType, self.ID, message=message, location=location, object=object)
        
        self.tasks.append(newTask)
        
    def removeTask(self, taskID):         
        for index, task in zip(self.tasks, range(len(self.tasks))):
            if task.taskID == taskID:
                break

        if index >= 0 or index < len(self.tasks):
            del self.tasks[index]


class ShortTermMemory:
    def __init__(self, visionModule):
        
        self.visionModule = visionModule
        
        self.peopleEncountered = {}   
        self.resetMemory()
        
        stefania = Person('stefania')
        alex = Person('alex')
        self.peopleEncountered['stefania'] = stefania
        self.peopleEncountered['alex'] = alex

        # am niste clasificatori impliciti pe care doar ii copiez in folderul ala
        # si pentru permanent si pentru short time memory
        self.model_temporary = None
        self.model_permanent = None
        p1 = subprocess.Popen(["cp", starterPermanentPeopleClassifier, permanentPeopleClassifier])
        p2 = subprocess.Popen(["cp", starterTemporaryPeopleClassifier, temporaryPeopleClassifier])
        p1.wait()
        p2.wait()
        
        self.createModels(temporary=True, permanent=True)

        
    def createModels(self, temporary=False, permanent=False):
        # Clasificator temporar
        if temporary:
            classifier_temporary_filename_exp = os.path.expanduser(temporaryPeopleClassifier)
            with open(classifier_temporary_filename_exp, 'rb') as infile:
                (self.model_temporary, class_names) = pickle.load(infile, encoding='latin1')
                print('load classifier file-> %s' % classifier_temporary_filename_exp)
        
        # Clasificator permanent
        if permanent:
            classifier_permanent_filename_exp = os.path.expanduser(permanentPeopleClassifier)
            with open(classifier_permanent_filename_exp, 'rb') as infile:
                (self.model_permanent, class_names) = pickle.load(infile, encoding='latin1')
                print('load classifier file-> %s' % classifier_permanent_filename_exp)
 
    def resetMemory(self):
        # TODO: poae astept in alta parte .... gen mai tarziu
        p1 = subprocess.Popen(["./ShortTermMemory/clean.sh"], shell=True)
        p1.wait()


    def addTemporaryPerson(self, some_faces, name, isInPermanent, some_bodies=None):
        
        # (name, isKnown) = self.isKnownPerson(some_faces, some_bodies)

        # if the name is already in the short term memory =>
        # give another index to the name
        # this is useful for people with the same name
        person_name = name
        if name in self.peopleEncountered.keys() or (
                name in PERMANENT_USERS and not isInPermanent):
            person_name = '%s%s' % (name, len(self.peopleEncountered))


        # sterg toate folderele ce se gasesc in folderul ala de aliniat chestii
        if not os.listdir(newPersonFacesFolder) == []:
            p1 = subprocess.Popen("rm -r %s/*" % (newPersonFacesFolder), shell=True)
            p1.wait()
        if not os.listdir(newPersonAlignedFacesFolder) == []:
            p1 = subprocess.Popen("rm -r %s/*" % (newPersonAlignedFacesFolder), shell=True)
            p1.wait()

        p1 = subprocess.Popen("mkdir %s/%s" % (newPersonFacesFolder, person_name), shell=True)
        p1.wait()
        p1 = subprocess.Popen("mkdir %s/%s" % (newPersonAlignedFacesFolder, person_name), shell=True)
        p1.wait()

        if isInPermanent: # the person is not in the short time memory, but is a permanent person
            # copiez folderul persoanei din permanent aligned folder in ala temporar
            # care are fete de aliniat
            p1 = subprocess.Popen("cp -r %s/%s %s/" % (permanentAlignedPeopleFolder,
                                                       person_name, newPersonAlignedFacesFolder),
                                  shell=True)
            p1.wait()
            print("Person in the permanent memory => put him/her in the short term memory!----------->")
        else:
            print("Person nowhere => put him/her in the short term memory!----------->")

        for (face, index) in zip(some_faces, range(len(some_faces))):
            cv2.imwrite("%s/%s/person%s.jpg" % (newPersonFacesFolder, person_name, index), face)

        p1 = subprocess.Popen("./ShortTermMemory/facenet_usage.sh %s %s %s %s" %
                              (newPersonFacesFolder, newPersonAlignedFacesFolder,
                               temporaryAlignedPeopleFolder, temporaryPeopleClassifier),
                              shell=True)
        # p1.wait()

        new_person = Person(person_name)
        self.peopleEncountered[person_name] = new_person
        self.createModels(temporary=True)


    # Verifica daca persoana e una cunoscuta: in short term memory sau permenent memory
    # nu mai folosesc functia asta pt ca determin dinainte chestiile astea
    def isKnownPerson(self, some_faces, some_bodies=None):
        # checks if the faces correspond with a known person probability
        # it uses face recognition and optionally body recognition

        # Iterez prin toate pozele si apelez identifyPerson si retin ce a intors majoritatea voturilor
        temporary_person_names = []
        permanent_person_names = []
        for frame in some_faces:
            people = classifyFacesFromFrame(self, copy.deepcopy(frame),
                                            self.model_temporary,
                                            self.model_permanent, show=False)

            if people[0][2] == True:
                temporary_person_names.append(people[0][1])
            elif people[0][2] == False and people[0][1] != '':
                permanent_person_names.append(people[0][1])
            else:
                temporary_person_names.append('')
                permanent_person_names.append('')

        try:
            temporary_person = mode(temporary_person_names)
        except StatisticsError:
            temporary_person = ''
        print(str(temporary_person_names) + " cu mode: " + temporary_person)

        try:
            permanent_person = mode(permanent_person_names)
        except StatisticsError:
            permanent_person = ''
        print(str(permanent_person_names) + " cu mode: " + permanent_person)

        # if the person is in the short time memory, it will return (person_name, true)
        # if the person is a permanent user, but is not in the short term memory returns (person_name, false)
        # if the person is neither a permanent user, nor in the short time memory returns (None, false)
        if temporary_person:
            return (temporary_person, True)
        elif permanent_person:
            return (permanent_person, False)
        else:
            return (None, False)