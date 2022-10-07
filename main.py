# OpenCV program to detect face in real time
# import libraries of python OpenCV
# where its functionality resides
import cv2

import RPi.GPIO as GPIO
import time
from AlphaBot2 import AlphaBot2
from servoSetup import PCA9685

import math

DR = 16
DL = 19

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
GPIO.setup(DL,GPIO.IN,GPIO.PUD_UP)

Ab = AlphaBot2()
setUpServoStart = PCA9685()

 
# Trained XML classifiers describes some features of some
# object we want to detect a cascade function is trained
# from a lot of positive(faces) and negative(non-faces)
# images.
cat_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Trained XML file for detecting eyes
'''
glass_cascade = cv2.CascadeClassifier('haarcascade_glasses.xml')
 '''
# capture frames from a camera
cap = cv2.VideoCapture(0)
horizontalTreshold = 50
idleLoopTrigerCounter = 0

def turnRight():
	Ab.right()
	time.sleep(0.15)
	return
	
def turnLeft():
	Ab.left()
	time.sleep(0.15)
	return
	
def chargeForwardMyGoodSir():
	print("I SAID GOOD DAY SIR")
	Ab.forward()
	time.sleep(0.15)
	return
	
def stopTheRobot():
    Ab.stop()
    return

def Distance():
    print("Something #1")
    GPIO.output(TRIG,GPIO.HIGH)
    time.sleep(0.000015)
    print("Something #2")
    GPIO.output(TRIG,GPIO.LOW)
    print("Something #3")
    while not GPIO.input(ECHO):
        print("Something #4")
        pass
    t1 = time.time()
    print("Something #5")
    while GPIO.input(ECHO):
        print("Something #6")
        pass
    t2 = time.time()
    print("Something #7")
    return (t2-t1)*34000/2

def drawRectangle(gray, img, cats):
    averageX = 0
    for (x,y,w,h) in cats:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        averageX = averageX + 1
    return averageX / len(cats) 
        	
 
# servo setup
setUpServoStart.setServoStartUp(1400)

# loop runs if capturing has been initialized.
while 1:
 
    # reads frames from a camera
    ret, img = cap.read()
     
    # convert to gray scale of each frames
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
    # Detects faces of different sizes in the input image
    
    cats = cat_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
    idleLoopTrigerCounter = idleLoopTrigerCounter + 1
    
    catIsDetected = len(cats) > 0
    if catIsDetected:
        #draw rectangular, move robot
        averageX = drawRectangle(gray, img, cats)
        print(f"AverageX {0}", averageX)
        xAbs = abs(240 - averageX)
        if xAbs < horizontalTreshold: 
            if averageX < 240:
                turnRight()
            if averageX > 240:
                turnLeft()
        else:
            DR_status = GPIO.input(DR)
            DL_status = GPIO.input(DL)
            if((DL_status == 0) or (DR_status == 0)):
                print("we stop")
                stopTheRobot()	
            else:
                chargeForwardMyGoodSir()
    else:
        idleLoopTrigerCounter = idleLoopTrigerCounter + 1
    '''    	
    for (x,y,w,h) in cats:
        # To draw a rectangle in a face
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        
        print("{0},  {1}.".format(x, y))
        xAbs = abs(240 - x)
        if xAbs < horizontalTreshold: 
            if x < 240:
                turnRight()
            if x > 240:
                turnLeft()
        else:
            DR_status = GPIO.input(DR)
            DL_status = GPIO.input(DL)
            if((DL_status == 0) or (DR_status == 0)):
                print("we stop")
                stopTheRobot()	
            else:
                chargeForwardMyGoodSir()
        
        time.sleep(0.2)
''' 
       
 
    # Display an image in a window
    cv2.imshow('img',img)
 
    # Wait for Esc key to stop
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
 
# Close the window
cap.release()
 
# De-allocate any associated memory usage
cv2.destroyAllWindows()
