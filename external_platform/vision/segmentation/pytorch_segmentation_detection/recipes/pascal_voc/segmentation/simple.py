from PIL import Image
import cv2

cap = cv2.VideoCapture(0) # says we capture an image from a webcam
_,cv2_im = cap.read()
cv2_im = cv2.cvtColor(cv2_im,cv2.COLOR_BGR2RGB)
pil_im = Image.fromarray(cv2_im)
pil_im.show()