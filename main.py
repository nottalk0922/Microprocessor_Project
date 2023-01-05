# importing modules
import cv2
import time
import numpy as np
import RPi.GPIO as GPIO
from PCA9685 import PCA9685
import dlib
from scipy.spatial import distance
import serial
from pantilthat import *

# ********************** Setting **********************
# cam_pan = 40
# cam_tilt = 20
# 
# pan(cam_pan-90)
# tilt(cam_tilt-90)
# light_mode(WS2812)
# setting Camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 200)

# setting PWM (Pan-tilt)
pwm = PCA9685()
print ("This is an PCA9685 routine")    
pwm.setPWMFreq(50)
pwm.setServoPulse(1,500) 
pwm.setRotationAngle(1, 0)

# Loading dlib
hog_face_detector = dlib.get_frontal_face_detector()
dlib_facelandmark = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Loading Haar Cascades classifier xml
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

# setting GPIO
LED = 18
LEFT_MOTOR_SIGNAL = 17
RIGHT_MOTOR_SIGNAL = 23
LEFT_END_SIGNAL = 27
RIGHT_END_SIGNAL = 22

GPIO.setmode(GPIO.BCM) # open the gpio chip
GPIO.setup(LED,GPIO.OUT)
GPIO.setup(LEFT_MOTOR_SIGNAL, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_SIGNAL, GPIO.OUT)
GPIO.setup(LEFT_END_SIGNAL, GPIO.IN)
GPIO.setup(RIGHT_END_SIGNAL, GPIO.IN)

# setting uartq
# ser = serial.Serial(
#     port = "/dev/ttyAMA3",
#     baudrate = 115200,
#     parity = serial.PARITY_NONE,
#     stopbits = serial.STOPBITS_ONE,
#     bytesize = serial.EIGHTBITS,
#     timeout = 1    
# )

# *********************** function ********************
# def lights(r,g,b,w):
#     for x in range(18):
#         set_pixel_rgbw(x,r if x in [3,4] else 0,g if x in [3,4] else 0,b,w if x in [0,1,6,7] else 0)
#     show()
# 
# lights(0,0,0,50)

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

    # Detect Face
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
            
        cv2.imshow('detect eyes', frame)
        left_ear = calculate_EAR(leftEye)
        right_ear = calculate_EAR(rightEye)

        EAR = (left_ear+right_ear)/2
        EAR = round(EAR,2)

        if EAR<0.19:
            print("you Sleep")
            return True
        
        return False

# def detectionFace(img):
#     yumu = 0
#     # transform img -> gray color image 
#     grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     # face detection
#     faces = face_cascade.detectMultiScale(grayImg, 1.3, 3)
    
#     for (x,y,w,h) in faces:
#         cv2.rectangle(img, (x,y), (x+w, y+h), (0,0,255),2 )
#         x = x + (w/2)
#         y = y + (h/2)
        
#         turn_x = float(x-160)
#         turn_y = float(y-100)
        
#         turn_x /= float(160)
#         turn_y /= float(100)
        
#         turn_x *= 2.5
#         turn_y *= 2.5
        
#         cam_pan   = -turn_x
#         cam_tilt  = turn_y

#         cam_pan = max(0,min(180,cam_pan))
#         cam_tilt = max(0,min(180,cam_tilt))
        
#         pan(int(cam_pan-90))
#         tilt(int(cam_tilt-90))
        
#         # pwm.setRotationAngle(1, turn_x)
#         # pwm.setRotationAngle(0, turn_y)
#     cv2.imshow('Detection face', img)
    
def led_on():
    GPIO.output(LED, GPIO.HIGH)
    
def led_off():
    GPIO.output(LED, GPIO.LOW)
        
def transmitSignal():
    if GPIO.input(LEFT_END_SIGNAL):
        GPIO.output(RIGHT_MOTOR_SIGNAL, GPIO.HIGH)
    if GPIO.input(RIGHT_END_SIGNAL):
        GPIO.output(LEFT_MOTOR_SIGNAL,GPIO.HIGH) 
    


# ********************** variable declaration ************************
# keyboard event processing variable
err = 1

# checking detect face result
detect = 0

# transmit data
data = 1

# count
cnt = 0
# ********************** action code ***************************
while True:
    detect = 0
    # Zeroing before starting
    pwm.setRotationAngle(1,10)
    time.sleep(1)
    print('start')
    # movement Pantilt and Video Capture 
    for i in range(10,150,1): 
        pwm.setRotationAngle(1, i)   
        pwm.setRotationAngle(0, 30)
        
        err, frame = CaptureCamera()
        if err == True:
            if detectionSleep(frame):
                cv2.imwrite("imgs/Sleep("+str(cnt)+").jpg",frame)
                cnt += 1
                led_on()
        else:
            break
        
        time.sleep(0.1)
    if err == False:
        break
    
    for i in range(170,10,-160): 
        pwm.setRotationAngle(1, i)   
        pwm.setRotationAngle(0, 30) 
        time.sleep(0.1)
        
    transmitSignal()

#close Program
print("\nProgram end")
pwm.exit_PCA9685()
GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()
