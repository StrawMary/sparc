
import copy
import cv2
import numpy as np
import os
from scipy import misc
import tensorflow as tf

from Vision.FaceNet.detect_face import detect_face
from Vision.FaceNet.facenet import to_rgb, prewhiten, flip
from ShortTermMemory.short_time_memory import temporaryAlignedPeopleFolder, permanentAlignedPeopleFolder

class FaceDetectRecog:
    def __init__(self, sess, pnet, rnet, onet):
    
        self.sess = sess
        self.pnet = pnet
        self.rnet = rnet
        self.onet = onet

    def identifyBoundingBoxes(self, frame, resize=True):
        minsize = 20  # minimum size of face
        threshold = [0.6, 0.7, 0.7]  # three steps's threshold
        factor = 0.709  # scale factor
        image_size = 182
        input_image_size = 160
        
        images_placeholder = tf.get_default_graph().get_tensor_by_name("input:0")
        embeddings = tf.get_default_graph().get_tensor_by_name("embeddings:0")
        phase_train_placeholder = tf.get_default_graph().get_tensor_by_name("phase_train:0")
        embedding_size = embeddings.get_shape()[1]

        if resize:
            frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)    #resize frame (optional)

        if frame.ndim == 2:
            frame = to_rgb(frame)
        frame = frame[:, :, 0:3]
        bounding_boxes, _ = detect_face(frame, minsize, self.pnet, self.rnet, self.onet, threshold, factor)
        nrof_faces = bounding_boxes.shape[0]
        #print('Detected_FaceNum: %d' % nrof_faces)

        if nrof_faces > 0:
        
            det = bounding_boxes[:, 0:4]
            img_size = np.asarray(frame.shape)[0:2]

            cropped = []
            scaled = []
            scaled_reshape = []
            bb = np.zeros((nrof_faces,4), dtype=np.int32)
            embedding_array = []
            
            for i in range(nrof_faces):
                emb_array = np.zeros((1, embedding_size))

                bb[i][0] = det[i][0]
                bb[i][1] = det[i][1]
                bb[i][2] = det[i][2]
                bb[i][3] = det[i][3]

                if bb[i][0] <= 0 or bb[i][1] <= 0 or bb[i][2] >= len(frame[0]) or bb[i][3] >= len(frame):
                    print('Face is inner of range!')
                    continue

                cropped.append(frame[bb[i][1]:bb[i][3], bb[i][0]:bb[i][2], :])
                cropped[i] = flip(cropped[i], False)
                scaled.append(misc.imresize(cropped[i], (image_size, image_size), interp='bilinear'))
                scaled[i] = cv2.resize(scaled[i], (input_image_size,input_image_size),
                                       interpolation=cv2.INTER_CUBIC)
                scaled[i] = prewhiten(scaled[i])
                scaled_reshape.append(scaled[i].reshape(-1,input_image_size,input_image_size,3))
                feed_dict = {images_placeholder: scaled_reshape[i], phase_train_placeholder: False}
                emb_array[0, :] = self.sess.run(embeddings, feed_dict=feed_dict)
                embedding_array.append(emb_array)
        else:
            return [], []
      
        return bb, embedding_array
        
        
    def classifyBox(self, emb_array, model, aligned_faces_dir):
    
        HumanNames = os.listdir(aligned_faces_dir)
        HumanNames.sort()
        personName = ''
        
        if len(HumanNames) == 2:
            face_treshold = 0.9
        else:
            face_treshold = 0.6    
        
        predictions = model.predict_proba(emb_array)
        #print("Predictions: ", str(predictions))
        best_class_indices = np.argmax(predictions, axis=1)
        #print("Best class indices: ", str(best_class_indices))
        best_class_probabilities = predictions[np.arange(len(best_class_indices)), best_class_indices]
                        
        #print("Best class probabilites: ", str(best_class_probabilities))
        #print("Human names: ", str(HumanNames))
        if best_class_probabilities[0] >= face_treshold:
            #print('Result (best_class_indices[0]): ', best_class_indices[0])
            #print("best_class_indices: ", str(best_class_indices))
            
            #print("Person identified")
            for H_i in HumanNames:
                if HumanNames[best_class_indices[0]] == H_i:
                    personName = HumanNames[best_class_indices[0]]
        
        return personName
        
        
    def classifyFacesFromFrame(self, initialFrame, model_temporary, model_permanent, show = False):
        # checks if the faces correspond with a known person probability
        # it uses face recognition and optionally body recognition     
       
       
        frame = copy.deepcopy(initialFrame)
        resize = False
        bb, embedding_array = self.identifyBoundingBoxes(frame, resize=resize)
        
        if resize:
            frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)

        people = []
        if bb != []:       
            for b, emb_array in zip(bb, embedding_array):
                personName1 = self.classifyBox(emb_array, model_temporary, temporaryAlignedPeopleFolder)
                personName2 = self.classifyBox(emb_array, model_permanent, permanentAlignedPeopleFolder)
                
                cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                text_x = b[0]
                text_y = b[3] + 20
                cv2.putText(frame, "P:%s, T:%s" % (personName2, personName1), (text_x, text_y), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                                1, (0, 0, 255), thickness=1, lineType=2)
                
                # if the person is in the short time memory, it will return (person_name, true)        
                # if the person is a permanent user, but is not in the short term memory returns (person_name, false)
                # if the person is neither a permanent user, nor in the short time memory returns (None, false)
                if personName1 != '':
                    people.append((b, personName1, True))
                elif personName2 != '':
                    people.append((b, personName2, False))
                else:
                    people.append((b, '', False))      
    
        if show:
            cv2.imshow('Frame', frame)   
            k = cv2.waitKey(1)

        #result.put((people, frame))
        return people, frame

    
    
"""
def identifyPerson(self, frame, model, aligned_faces_dir, resize=False):    
            
    minsize = 20  # minimum size of face
    threshold = [0.6, 0.7, 0.7]  # three steps's threshold
    factor = 0.709  # scale factor
    image_size = 182
    input_image_size = 160
    
    HumanNames = os.listdir(aligned_faces_dir)
    HumanNames.sort()
    
    (isPersonIdentified, personName) = (False, "")
    
    if len(HumanNames) == 2:
        face_treshold = 0.9
    else:
        face_treshold = 0.6
    
    images_placeholder = tf.get_default_graph().get_tensor_by_name("input:0")
    embeddings = tf.get_default_graph().get_tensor_by_name("embeddings:0")
    phase_train_placeholder = tf.get_default_graph().get_tensor_by_name("phase_train:0")
    embedding_size = embeddings.get_shape()[1]

    if resize:
        frame = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)    #resize frame (optional)

    if frame.ndim == 2:
        frame = facenet.to_rgb(frame)
    frame = frame[:, :, 0:3]
    bounding_boxes, _ = detect_face.detect_face(frame, minsize, self.pnet, self.rnet, self.onet, threshold, factor)
    nrof_faces = bounding_boxes.shape[0]
    #print('Detected_FaceNum: %d' % nrof_faces)

    if nrof_faces > 0:
        
        isPersonIdentified = True
    
        det = bounding_boxes[:, 0:4]
        img_size = np.asarray(frame.shape)[0:2]

        cropped = []
        scaled = []
        scaled_reshape = []
        bb = np.zeros((nrof_faces,4), dtype=np.int32)

        for i in range(nrof_faces):
            emb_array = np.zeros((1, embedding_size))

            bb[i][0] = det[i][0]
            bb[i][1] = det[i][1]
            bb[i][2] = det[i][2]
            bb[i][3] = det[i][3]

            # inner exception
            if bb[i][0] <= 0 or bb[i][1] <= 0 or bb[i][2] >= len(frame[0]) or bb[i][3] >= len(frame):
                print('Face is inner of range!')
                continue

            cropped.append(frame[bb[i][1]:bb[i][3], bb[i][0]:bb[i][2], :])
            cropped[i] = facenet.flip(cropped[i], False)
            scaled.append(misc.imresize(cropped[i], (image_size, image_size), interp='bilinear'))
            scaled[i] = cv2.resize(scaled[i], (input_image_size,input_image_size),
                                   interpolation=cv2.INTER_CUBIC)
            scaled[i] = facenet.prewhiten(scaled[i])
            scaled_reshape.append(scaled[i].reshape(-1,input_image_size,input_image_size,3))
            feed_dict = {images_placeholder: scaled_reshape[i], phase_train_placeholder: False}
            emb_array[0, :] = self.sess.run(embeddings, feed_dict=feed_dict)
            predictions = model.predict_proba(emb_array)
            #print("Predictions: ", str(predictions))
            best_class_indices = np.argmax(predictions, axis=1)
            #print("Best class indices: ", str(best_class_indices))
            best_class_probabilities = predictions[np.arange(len(best_class_indices)), best_class_indices]
                            
            #print("Best class probabilites: ", str(best_class_probabilities))
            #cv2.rectangle(frame, (bb[i][0], bb[i][1]), (bb[i][2], bb[i][3]), (0, 255, 0), 2)    #boxing face

            #print("Human names: ", str(HumanNames))
            if best_class_probabilities[0] >= face_treshold:
                #plot result idx under box
                #text_x = bb[i][0]
                #text_y = bb[i][3] + 20
                #print('Result (best_class_indices[0]): ', best_class_indices[0])
                #print("best_class_indices: ", str(best_class_indices))
                print("Person identified")
                
                for H_i in HumanNames:
                    if HumanNames[best_class_indices[0]] == H_i:
                        result_names = HumanNames[best_class_indices[0]]
                        
                        #cv2.putText(frame, result_names, (text_x, text_y), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        #            1, (0, 0, 255), thickness=1, lineType=2)
                        personName = copy.deepcopy(result_names)
                        # print("Result name este -------------> " + result_names)
                        # print("am pus numele ----------------> ", personName)
            else:
                print("Person NOT identified.")
                personName = ""
            
            print()
            break        
    else:
        print('Unable to align')

    #cv2.imshow('Frame', frame)
    '''
    k = cv2.waitKey(1)
    if k == 27: 
        cv2.destroyAllWindows()
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite("identif.jpg", frame)
        cv2.destroyAllWindows()
    '''        
    return (isPersonIdentified, personName), bb[i]
"""