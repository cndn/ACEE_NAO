# -*- encoding: UTF-8 -*-

import os
import sys
import time
import cv2
import hough
import socket
import ftplib
from naoqi import ALProxy
import numpy as np
from modulewalk import move2
from modulewalklr import move3
from modulewalkr import move4
from onepace import leftPace,rightPace
IP = "192.168.0.105"
#IP = "127.0.0.1"
PORT = 9559

def takeBallPic(fileName):
    global IP
    global PORT
    try:
      photoCaptureProxy = ALProxy("ALPhotoCapture", IP, PORT)
    except Exception, e:
      print "Error when creating ALPhotoCapture proxy:"
      print str(e)
      exit(1)
    photoCaptureProxy.setResolution(2)
    photoCaptureProxy.setPictureFormat("jpg")
    a = photoCaptureProxy.takePictures(1, "/home/nao/recordings/kickball", 'balltest')
    

def getFTP(fileName):
    global IP
    DIRN = "recordings/kickball"
    HOST = IP
    FILE = fileName 
    try:  
        f = ftplib.FTP(HOST)  
    except (socket.error, socket.gaierror):  
        print 'ERROR:cannot reach " %s"' % HOST  
        return  
    print '***Connected to host "%s"' % HOST  
  
    try:  
        f.login('nao','nao')  
    except ftplib.error_perm:  
        print 'ERROR: cannot login anonymously'  
        f.quit()  
        return  
    print '*** Logged in as "anonymously"'  
    try:  
        f.cwd(DIRN)  
    except ftplib.error_perm:  
        print 'ERRORL cannot CD to "%s"' % DIRN  
        f.quit()  
        return  
    print '*** Changed to "%s" folder' % DIRN  
    try: 
        f.retrbinary('RETR %s' % FILE, open(FILE, 'wb').write)  
    except ftplib.error_perm:  
        print 'ERROR: cannot read file "%s"' % FILE  
        os.unlink(FILE)  
    else:  
        print '*** Downloaded "%s" to CWD' % FILE  
    f.quit()  
    return

def detectBall(fileName):
    circles = hough.detectCircle(fileName)
    print circles
    return circles

def computeLocation(circles):
    x = 340.0-circles[0][0][0]   #340
    X = 280.0/(circles[0][0][1]-138)
    Y = x*X/568.0
    
    ballLocation = np.array([X,Y])
    ballLocation = ballLocation.tolist()
    ballLocation.append(0)
    print ballLocation
    return ballLocation

def moveToBall(ballLocation):
    global IP
    global PORT
    x = ballLocation[0]
    y = ballLocation[1]
    stepLength = 0.06
    pace = int(x/stepLength) + 2 # 2 more steps
    move2(IP,pace,stepLength)

 
def seeLoop():
    postureProxy.goToPosture("StandInit", 1.0)
    time.sleep(2)
    fileName= 'balltest.jpg'
    takeBallPic(fileName)
    getFTP(fileName)
    circles = detectBall(fileName)
    ballLocation = computeLocation(circles)
    print ballLocation
    x = ballLocation[0]
    y = ballLocation[1]
    stepLength = 0.05
    pace1 = int(y/(stepLength))*2
    move3(IP,pace1,stepLength)
    while abs(y)>stepLength/2:
        postureProxy.goToPosture("StandInit", 1.0)
        time.sleep(2)
        fileName= 'balltest.jpg'
        takeBallPic(fileName)
        getFTP(fileName)
        circles = detectBall(fileName)
        ballLocation = computeLocation(circles)
        print ballLocation
        x = ballLocation[0]
        y = ballLocation[1]
        stepLength = 0.05
        pace1 = int(y/stepLength)*2
        if pace1<0:
            print pace1
            move4(IP,-pace1,stepLength)
        elif pace1>0:
            print pace1
            move3(IP,pace1,stepLength)
        else:
            break
    stepLength = 0.06
    pace2 = int(x/stepLength)+1
    move2(IP,pace2,stepLength)
    pace = 1
    stepLength = 0.06
    leftPace(IP,pace,stepLength)
    rightPace(IP,pace,stepLength)

if __name__ == "__main__":
    try:
        postureProxy = ALProxy("ALRobotPosture", IP, PORT)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e
    # Send NAO to Pose Init
    #postureProxy.goToPosture("Stand", 1.0)
    seeLoop()
    
