from utils import *
from darknet import Darknet
import cv2

def demo(cfgfile, weightfile):
    m = Darknet(cfgfile)
    m.load_weights(weightfile)
    print('Loading weights from %s... Done!' % (weightfile))

    classes = 80

    if classes == 20:
        namesfile = 'cfg/voc.names'
    elif classes == 80:
        namesfile = 'cfg/coco.names'
    else:
        namesfile = 'cfg/names'
    class_names = load_class_names(namesfile)
 
    use_cuda = 1
    if use_cuda:
        m.cuda()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Unable to open camera")
        exit(-1)

    while True:
        res, img = cap.read()
        if res:
            sized = cv2.resize(img, (m.width, m.height))
            bboxes = detect(m, sized, 0.5, 0.4, use_cuda)
            print('------')
            draw_img = plot_boxes_cv2(img, bboxes, None, class_names)
            cv2.imshow(cfgfile, draw_img)
            cv2.waitKey(1)
        else:
             print("Unable to read image")
             exit(-1) 

############################################
if __name__ == '__main__':
    if len(sys.argv) == 3:
        cfgfile = sys.argv[1]
        weightfile = sys.argv[2]
        demo(cfgfile, weightfile)
        #demo('cfg/tiny-yolo-voc.cfg', 'tiny-yolo-voc.weights')
    else:
        print('Usage:')
        print('    python demo.py cfgfile weightfile')
        print('')
        print('    perform detection on camera')
