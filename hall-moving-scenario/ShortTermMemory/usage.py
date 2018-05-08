# Usage on camera


import short_time_memory
import tensorflow as tf
import cv2


def main():

    with tf.Graph().as_default():
        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.6)
        sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=False))
        with sess.as_default():
            modeldir = './facenet/pre_model/20170511-185253.pb'
            facenet.load_model(modeldir)

            pnet, rnet, onet = detect_face.create_mtcnn(sess, './facenet/d_npy')
            stm = short_time_memory.ShortTermMemory(sess, pnet, rnet, onet)
            
            cam = cv2.VideoCapture(0)
            while True:
                ret_val, img = cam.read()               
                people = stm.classifyFacesFromFrame(img)   
          

if __name__ == '__main__':
    main()