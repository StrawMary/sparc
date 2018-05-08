# Short time memory test file

import short_time_memory
import tensorflow as tf
import facenet
import detect_face
import cv2
import sys

def test_encounterActions():
    with tf.Graph().as_default():
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)
        sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=False))
        with sess.as_default():
            
            print('Loading feature extraction model')
            modeldir = './facenet/pre_model/20170511-185253.pb'
            facenet.load_model(modeldir)

            pnet, rnet, onet = detect_face.create_mtcnn(sess, './facenet/d_npy')
            
            stm = short_time_memory.ShortTermMemory(sess, pnet, rnet, onet)

            # Caz 1: Stafania, Alex care e in temporary memory => nu fac nimic            
            # citesc pozele cu Stefania din fisierul de test
            
            
            stefania_list = []
            for i in range(1, 4):
                stefania_list.append(cv2.imread('./test_dir/stefania_test%s.jpg' % (i), 1))
            # citesc pozele cu Alex din fisierul de test
            alexg_list = []
            for i in range(1, 3):
                alexg_list.append(cv2.imread('./test_dir/alexg_test%s.jpg' % (i), 1))
                
            stm.encounterActions(stefania_list)
            stm.encounterActions(alexg_list)
            
            
            # Caz 2: Miruna care e in permanent memory -> adaug in temporary 
            # citesc pozele cu Miruna din fisierul de test
            miruna_list = []
            for i in range(1, 3):
                miruna_list.append(cv2.imread('./test_dir/miruna_test%s.jpg' % (i), 1))
            stm.encounterActions(miruna_list)
            
            
            # Caz 3: Alex Sorici care nu e nicaieri -> adaug in permanent
            # citesc fisierele cu alexs din fisierul de test
            alexs_list = []
            for i in range(1, 13):
                alexs_list.append(cv2.imread('./test_dir/alexs/alexs_test%s.jpg' % (i), 1))
            stm.encounterActions(alexs_list)
            
            # Caz 4: test again if the robot knows Miruna
            stm.encounterActions(miruna_list)
        
        
def test_isKnownPerson():
    with tf.Graph().as_default():
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)
        sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=False))
        with sess.as_default():
            
            print('Loading feature extraction model')
            modeldir = './facenet/pre_model/20170511-185253.pb'
            facenet.load_model(modeldir)

            pnet, rnet, onet = detect_face.create_mtcnn(sess, './facenet/d_npy')
            
            stm = short_time_memory.ShortTermMemory(sess, pnet, rnet, onet)
            
            image_list1 = []
            image_list1.append((cv2.imread('./test_dir/stefania_test1.jpg', 1), "stefania ghita"))
            image_list1.append((cv2.imread('./test_dir/stefania_test2.jpg', 1), "stefania ghita"))
            image_list1.append((cv2.imread('./test_dir/stefania_test3.jpg', 1), "stefania ghita"))
            image_list1.append((cv2.imread('./test_dir/mihait_test1.jpg', 1), "mihai trascau"))
            image_list1.append((cv2.imread('./test_dir/miruna_test1.jpg', 1), "miruna barbu"))
            
            
            image_list2 = []
            image_list2.append((cv2.imread('./test_dir/mihait_test1.jpg', 1), "mihai trascau"))
            image_list2.append((cv2.imread('./test_dir/miruna_test1.jpg', 1), "miruna barbu"))
            image_list2.append((cv2.imread('./test_dir/miruna_test2.jpg', 1), "miruna barbu"))
            
            image_list3 = []
            image_list3.append((cv2.imread('./test_dir/alexg_test2.jpg', 1), "alex gavril"))
            image_list3.append((cv2.imread('./test_dir/alexs_test1.jpg', 1), "alex sorici"))
            image_list3.append((cv2.imread('./test_dir/alexs_test2.jpg', 1), "alex sorici"))
            image_list3.append((cv2.imread('./test_dir/mihait_test1.jpg', 1), "mihai trascau"))
            image_list3.append((cv2.imread('./test_dir/miruna_test1.jpg', 1), "miruna barbu"))
            

            (personName1, isInShortMemory1) = stm.isKnownPerson([t[0] for t in image_list1])
            print("Trebuie afisat: stefania si true: %s, %s" % (personName1, isInShortMemory1))
            sys.stdout.flush()
            
            
            (personName2, isInShortMemory2) = stm.isKnownPerson([t[0] for t in image_list2])
            print("Trebuie afisat: miruna si false: %s, %s" % (personName2, isInShortMemory2))
            sys.stdout.flush()
            
            (personName3, isInShortMemory3) = stm.isKnownPerson([t[0] for t in image_list3])
            print("Trebuie afisat: None si false: %s, %s" % (personName3, isInShortMemory3))
            sys.stdout.flush()
            

def test_identifyPerson():
    
    with tf.Graph().as_default():
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)
        sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=False))
        with sess.as_default():
            
            print('Loading feature extraction model')
            modeldir = './facenet/pre_model/20170511-185253.pb'
            facenet.load_model(modeldir)

            pnet, rnet, onet = detect_face.create_mtcnn(sess, './facenet/d_npy')
            
            stm = short_time_memory.ShortTermMemory(sess, pnet, rnet, onet)
            
            # Clasificator temporar
            classifier_temporary_filename_exp = os.path.expanduser(temporaryPeopleClassifier)
            with open(classifier_temporary_filename_exp, 'rb') as infile:
                (model_temporary, class_names) = pickle.load(infile)
                print('load classifier file-> %s' % classifier_temporary_filename_exp)
                
            # Clasificator permanent
            classifier_permanent_filename_exp = os.path.expanduser(permanentPeopleClassifier)
            with open(classifier_permanent_filename_exp, 'rb') as infile:
                (model_permanent, class_names) = pickle.load(infile)
                print('load classifier file-> %s' % classifier_permanent_filename_exp)
            
            image_list = []
            image_list.append((cv2.imread('./test_dir/alexg_test1.jpg', 1), "alex gavril"))
            image_list.append((cv2.imread('./test_dir/alexg_test2.jpg', 1), "alex gavril"))
            image_list.append((cv2.imread('./test_dir/alexs_test1.jpg', 1), "alex sorici"))
            image_list.append((cv2.imread('./test_dir/alexs_test2.jpg', 1), "alex sorici"))
            image_list.append((cv2.imread('./test_dir/mihait_test1.jpg', 1), "mihai trascau"))
            image_list.append((cv2.imread('./test_dir/miruna_test1.jpg', 1), "miruna barbu"))
            image_list.append((cv2.imread('./test_dir/miruna_test2.jpg', 1), "miruna barbu"))
            image_list.append((cv2.imread('./test_dir/stefania_test1.jpg', 1), "stefania ghita"))
            image_list.append((cv2.imread('./test_dir/stefania_test2.jpg', 1), "stefania ghita"))
            image_list.append((cv2.imread('./test_dir/stefania_test3.jpg', 1), "stefania ghita"))
            
            print("Test permanent people classifier -----------------------")
            for (frame, name) in image_list:
                print("Image name -------------------------------------------> ", name)
                (isPersonIdentified, personName) = stm.identifyPerson(frame, model_permanent, permanentAlignedPeopleFolder)
                print("Is person identified: %s ... personName: %s." % (isPersonIdentified, personName))
                print("Image name final -------------------------------------> ", name)
                print("")
                sys.stdout.flush()
        
            print()
            print("Test temporary people classifier -----------------------")
            for (frame, name) in image_list:
                print("Image name -------------------------------------------> ", name)
                (isPersonIdentified, personName) = stm.identifyPerson(frame, model_temporary, temporaryAlignedPeopleFolder)
                print("Is person identified: %s ... personName: %s." % (isPersonIdentified, personName))
                print("Image name final -------------------------------------> ", name)
                print("")
                sys.stdout.flush()
