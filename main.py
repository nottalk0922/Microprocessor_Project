# importing modules
import cv2
import time
import numpy as np
import RPi.GPIO as GPIO
from PCA9685 import PCA9685
import dlib
from scipy.spatial import distance
# ********************** Setting **********************

# setting Camera
cap = cv2.VideoCapture(0)

# setting PWM (Pan-tilt)
pwm = PCA9685()
print ("This is an PCA9685 routine")    
pwm.setPWMFreq(50)
pwm.setServoPulse(1,500) 
pwm.setRotationAngle(1, 0)

# Loading dlib 
hog_face_detector = dlib.get_frontal_face_detector()
dlib_facelandmark = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
# *********************** function ********************

# Camera Capture function
def CaptureCamera():
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
        return False, frame
    return True, frame

# calculate ear distance
def calculate_EAR(eye): 
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    ear_aspect_ratio = (A+B)/(2.0*C)
    return ear_aspect_ratio

def detectionSleep(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = hog_face_detector(gray)
    for face in faces:

        face_landmarks = dlib_facelandmark(gray, face)
        leftEye = []
        rightEye = []

        # Detect right eye
        for n in range(36,42):
            x = face_landmarks.part(n).x
            y = face_landmarks.part(n).y
            leftEye.append((x,y))
            next_point = n+1
            if n == 41:
                next_point = 36
            x2 = face_landmarks.part(next_point).x
            y2 = face_landmarks.part(next_point).y
            cv2.line(frame,(x,y),(x2,y2),(0,255,0),1)

        # Detect left eye
        for n in range(42,48):
            x = face_landmarks.part(n).x
            y = face_landmarks.part(n).y
            rightEye.append((x,y))
            next_point = n+1
            if n == 47:
                next_point = 42
            x2 = face_landmarks.part(next_point).x
            y2 = face_landmarks.part(next_point).y
            cv2.line(frame,(x,y),(x2,y2),(0,255,0),1)
        print("check")
        left_ear = calculate_EAR(leftEye)
        right_ear = calculate_EAR(rightEye)

        EAR = (left_ear+right_ear)/2
        EAR = round(EAR,2)

        if EAR<0.19:
            print("you Sleep")


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
            detectionSleep(frame)
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
