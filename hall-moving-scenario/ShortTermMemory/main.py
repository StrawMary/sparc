
import cv2
import pickle
import sys
import tensorflow as tf

from Vision.FaceNet.face_det_rcog import FaceDetectRecog
from TaskManagement.tasksManagement import TriggerGenerator
from Vision.FaceNet.detect_face import create_mtcnn
from Vision.FaceNet.facenet import load_model

from ShortTermMemory.short_time_memory import ShortTermMemory

TASK_TYPES = {'say_something':1, 'take_me_to':2, 'find_me_object':2}
TASK_NO = 0
        

class PepperLife:
    def __init__(self, sess, pnet, rnet, onet):
        self.visionModule = FaceDetectRecog(sess, pnet, rnet, onet)
        self.shortTimeMemory = ShortTermMemory(self.visionModule)
        self.triggerGenerator = TriggerGenerator()
        
        # astea se pot intampla unitar, fara sa se intrerupa una pe alta
        self.tasksQueue = []
        # astea se pot petrece si in timp ce duce o persoana undeva
        self.saySomethingTasks = []
        
        self.currentTask = None
        self.onHoldTask = None
        

    def alive(self):
        # TODO: de schimbat cu streamul de la Pepper -------------------------------------
        cam = cv2.VideoCapture(0)
        # --------------------------------------------------------------------------------
        
        frameNo = 0
        while True:
            # TODO: de schimbat cu streamul de la Pepper -------------------------------------
            ret_val, img = cam.read() 
            # --------------------------------------------------------------------------------
            
            people = self.visionModule.classifyFacesFromFrame(img, self.shortTimeMemory.model_temporary, self.shortTimeMemory.model_permanent)
            
            triggeredLocations = self.triggerGenerator.qrCodesTrigger(img)
            if triggeredLocations != []:
                print("Location trigger: %s --------------------------" % (triggeredLocations))
            
            triggeredPeople = self.triggerGenerator.facesTrigger(people)
            if triggeredPeople != []:
                print("Person trigger: %s ----------------------------" % (triggeredPeople))

            print(frameNo)
            frameNo += 1
            sys.stdout.flush()
 
    def answerFromPepper(self, question):
        return input("Pepper, you have to answer: " + question + " .... ")
            
 
    def addTask(self, person_id, taskType, message=None, location=None, object=None):
        for name, person in self.peopleEncountered.items():
            if person.ID == person_id:
                person.addTask(taskType, message=message, location=location, object=object)
        
  
    def removeTask(self, person_id, task_id):
        for name, person in self.peopleEncountered.items():
            if person.ID == person_id:
                removeTask(self, task_id)
        
 
    def communicate(self, person):
        # Here a NLP module will be used to make Pepper-Human communication possible
        # This method will send replicas to Pepper to say and receive back messages from it
        
        # aici mai intai iau vechile cnvrobiri (dintr-un fisier pickle daca exista) daca a avut
        # sa fiu atenta ca e posibil ca ca fisierul sa fie gol ...

        dialogue = pickle.load(open("%s/%s.pkl" % (tasksFolder, person.name), "rb"))
        
        print("Hi %s!" % (person.name))
        while True:
            question = input("%s, ask Pepper a question: " % (person.name))
            if question == "?":
                break
            answer = self.answerFromPepper(question)
            print("Pepper's answer: %s" % (answer))
            
        # Dupa ce termin de vorbit, salvez fisierul cu conversatia
        pickle.dump(dialogue, open("%s/%s.pkl" % (tasksFolder, person.name), "wb"))    
        print("Am terminat de vorbt cu %s!------------------------------------------------" % (person.name))
        print()
        
    
    def getNameAndFacesFromPepper(self):
        person_name = input('Enter the name of the person: ')
        return [], person_name



if __name__ == "__main__":
    '''
    from PIL import Image
    print(decode(Image.open('./qr_codes/bathroom.png')))
    
    
    qrCodesDetector = qrtools.QR()
    qrCodesDetector.decode('./qr_codes/bathroom.png')
    print(self.qrCodesDetector.data, "-----------------------------------------------------")
    sys.stdout.flush()
    '''
       

    with tf.Graph().as_default():
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)
        sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=False))
        with sess.as_default():
            modeldir = './facenet/pre_model/20170511-185253.pb'
            load_model(modeldir)
            pnet, rnet, onet = create_mtcnn(sess, './facenet/d_npy')
            
            pepper = PepperLife(sess, pnet, rnet, onet)        
            pepper.alive()

   
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
#short_time_memory_test.test_isKnownPerson()
#short_time_memory_test.test_encounterActions()

#p1 = subprocess.Popen(['/c//cygwin64//bin//bash.exe', '/c//Users//MirunaBarbu//Desktop//proiect//pepper//short_term_memory//clean.sh'], shell=True)
#p1.wait()

#subprocess.call(["clean.sh"], shell=True)

# Pepper se duce spre grupuri de persoane ca sa vorbeasca cu ele
# asta o sa fie apelata cand pepper da de o persoana noua
# o sa primeasca ca input niste poze cu fata persoanei si cu corpul
# penru recunoastere in SVM

'''
stm = ShortTermMemory()

some_faces = [] # get_faces_from_pepper()
some_bodies = None # get_bodies_from_pepper()

stm.encounterActions(some_faces, some_bodies)
'''