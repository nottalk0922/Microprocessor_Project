import cv2
import numpy as np
from detectionSleep import *
import dlib


# Set camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320) 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,200)

# Define dlib model
hog_face_detector = dlib.get_frontal_face_detector()
dlib_facelandmark = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

while True:
    ret, img = cap.read()
    cv2.imshow('frame', img)
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', gray)
    
    faces = hog_face_detector(gray)
        
    detectionSleep(face, img, dlib_facelandmark)
    
    cv2.imshow('img',img)      
      
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

cv2.destroyAllWindows()