import cv2
import numpy as np

#Loading haar cascade
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

# Set camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320) 
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,200)


def detectionSleep(img):
        
    eyes = eye_cascade.detectMultiScale(img)
    for (ex, ey, ew, eh) in eyes:
        cv2.rectangle(img, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)


while True:
    ret, img = cap.read()
    cv2.imshow('frame', img)
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', gray)
    
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    #face detection
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        cv2.imshow("roi_gray",roi_gray)
        
        detectionSleep(roi_gray)
    
    cv2.imshow('img',img)      
      
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

cv2.destroyAllWindows()