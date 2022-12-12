# importing modules
import cv2
import time
import numpy as np
import RPi.GPIO as GPIO
from PCA9685 import PCA9685

# ********************** Setting **********************

# setting Camera
cap = cv2.VideoCapture(0)

# setting PWM (Pan-tilt)
pwm = PCA9685()
print ("This is an PCA9685 routine")    
pwm.setPWMFreq(50)
pwm.setServoPulse(1,500) 
pwm.setRotationAngle(1, 0)

# Loading Haar Cascades classifier xml
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

# *********************** function ********************

# Camera Capture function
def CaptureCamera():
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
        return False, frame
    return True, frame

def detectionFace(img):
    yumu = 0
    # transform img -> gray color image 
    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # face detection
    faces = face_cascade.detectMultiScale(grayImg, 1.3, 3)
    print(1)
    for (x,y,w,h) in faces:
        cv2.rectangle(img, (x,y), (x+w, y+h), (0,0,255),2 )
        roi_grayImg = grayImg[y:y+h, x:x+w]
        return 1,roi_grayImg    

    if yumu == 0:
        return 0,grayImg
    cv2.imshow('Detection face', img)

def detectionEye(img):
    eyes = eye_cascade.detectMultiScale(img)
    for (x,y,w,h) in eyes:
        cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0),2)
        print ('detect eye')
    cv2.imshow('Detection Eye', img)
    
    
# ********************** variable declaration ************************

# keyboard event processing variable
err = 1

# checking detect face result
detect = 0
# ********************** acion code ***************************

while True:
    detect = 0
    # Zeroing before starting
    pwm.setRotationAngle(1,170)
    pwm.setRotationAngle(0, 170)
    time.sleep(1)
    print('start')
    
    # movement Pantilt and Video Capture 
    for i in range(10,170,1): 
        pwm.setRotationAngle(1, i)   
        pwm.setRotationAngle(0, 10)
        
        err, frame = CaptureCamera()
        if err == True:
            detect, roi_frame = detectionFace(frame)
            if detect == 1:
                cv2.imshow('roi',roi_frame)
                detectionEye(roi_frame)
        else:
            break
        
        time.sleep(0.1)
    if err == False:
        break
    print(1)
    for i in range(170,10,-1): 
        pwm.setRotationAngle(1, i)   
        pwm.setRotationAngle(0, i) 
        if CaptureCamera() == False:
            err = False
            break
        time.sleep(0.1)
    if err == False:
        break
    print(2)

# close Program
pwm.exit_PCA9685()
print("\nProgram end")
cap.release()
cv2.destroyAllWindows()

