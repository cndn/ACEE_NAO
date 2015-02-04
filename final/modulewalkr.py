# -*- encoding: UTF-8 -*-

import sys
import time
from naoqi import ALProxy
import almath
import motion
import math
import numpy as np
from jacobian import *
from scipy.integrate import *
from massPointCalc import *

increment = 5
def compute(x,y,z,lorr):
    target = np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
    #print target
    return jacobianInverse(target,lorr)

def timeAppend(list1,list2):
    returnlist = []
    if len(list1) == 0:
        return list2
    for i in range(len(list1)):
        returnlist.append(list1[i])
        returnlist[i].extend(list2[i])
    return returnlist

def legMoveinWorld(leg,stepLength,Tsup): #consider stepLength
    global increment
    timeList = [(float(j)+1)/increment*Tsup for j in range(increment)]
    if leg == 'r':
        x = [0  for t in timeList]
        y = [ - stepLength/Tsup*1000 * 2*t - 50 for t in timeList[0:increment/2]]
        y.extend([- 90 for t in timeList[increment/2:increment]])
        z = [-306 + 40/Tsup * t for t in timeList[0:increment/4]]
        z.extend([-287 -40./Tsup*t for t in timeList[increment/4:increment/2]])
        z.extend([-287 -20 for t in timeList[increment/2:increment]])
    if leg == 'l':
        x = [0  for t in timeList]
        y = [10  for t in timeList[0:increment/2]]
        y.extend([-(stepLength/Tsup*1000*2*(t-0.5*Tsup) + 10)  for t in timeList[increment/2:increment]])
        #10?
        z = [-306 + 40/Tsup * t for t in timeList[0:increment/4]]
        z.extend([-287 -40./Tsup*t for t in timeList[increment/4:increment/2]])
        z.extend([-287 -20 for t in timeList[increment/2:increment]])
    return (x,y,z)

    #306!=267+40 but donnot worry><
    #310 is aslo a approximate value donnot worry either><

def legTrajectory(step,stepLength,Tsup):  #step = 0 left move
    global increment
    timeList = [(float(j)+1)/increment*Tsup for j in range(increment)]
#----------------------------step 0-------------------------------------
    if step == 0:
        '''
        1. trajectory of left leg in world
        '''
        legWorld = legMoveinWorld('r',stepLength,Tsup)
        x = legWorld[0]
        y = legWorld[1]
        z = legWorld[2]
        print x
        beginx = -stepLength/2##??
        px = stepLength/2
        '''
        2. trajectory of mass point in world
        '''
        #xMass = xsolution(beginx,px,Tsup,timeList) 
        #yMass = ysolution(step,Tsup,timeList)
        xMass = [ 0 for t in timeList]
        #yMass = [stepLength*0.5*t/Tsup for t in timeList]
        yMass = [(-stepLength*t/Tsup+0.05*math.sin(t/Tsup*3.14)) for t in timeList]
        x0 = [x[i]-xMass[i]*1000 for i in range(len(x))] #
        y0 = [y[i]-yMass[i]*1000 for i in range(len(y))]
        z0 = z
        left = []
        print "t:     "
        print timeList
        
        
        print "y 0:     "
        print y
        print "yMass 0:    "
        print yMass
        
        '''
        3. inverse kinematics for left leg
        '''
        for i in range(increment):
            left.extend(compute(x0[i],y0[i],z0[i],'r').transpose())
        left = np.transpose(left)
        left = left.tolist()
        '''
        4. trajectory of right leg in world
        '''
        x1 = [0 for i in x]
        y1 = [50 for i in y]
        z1 = [-306 for i in z]
        x1 = [x1[i]-xMass[i]*1000 for i in range(len(x1))]
        y1 = [y1[i]-yMass[i]*1000 for i in range(len(y1))]
        right = []
        '''
        5. inverse kinematics for right leg
        '''
        for i in range(increment):
            right.extend(compute(x1[i],y1[i],z1[i],'l').transpose())
        right = np.transpose(right)
        right = right.tolist()
        
        return (right,left)
#------------------------------step 1 ---------------------------------------
    if step == 1:
        legWorld = legMoveinWorld('l',stepLength,Tsup)
        x = legWorld[0]
        y = legWorld[1]
        z = legWorld[2]

        beginx = -stepLength/2 ##??
        px = stepLength/2
        
        #xMass = xsolution(beginx,px,Tsup,timeList)
        #yMass = ysolution(step,Tsup,timeList)
        
        xMass = [ 0 for t in timeList]
        #yMass = [stepLength*0.5*t/Tsup+0.07 for t in timeList]
        yMass = [-(stepLength*t/Tsup+stepLength*0.8 +0.05*math.sin(t/Tsup*3.14)) for t in timeList]
        #0.055 is not only to move masspoint
        # but aslo for move it over the foot ZMP
        x0 = [x[i]-xMass[i]*1000 for i in range(len(x))]
        y0 = [y[i]-yMass[i]*1000 for i in range(len(y))]
        z0 = z
        right = []
        for i in range(increment):
            right.extend(compute(x0[i],y0[i],z0[i],'l').transpose())
        right = np.transpose(right)
        right = right.tolist()
        print "t step1:     "
        print timeList
        print "y0 step1:     "
        print y0
        print "yMass step1:    "
        print yMass
        
        
        x1 = [0 for i in x]
        y1 = [-50-stepLength*1000*1.5 for i in y]
        z1 = [-306 for i in z]
        x1 = [x1[i]-xMass[i]*1000 for i in range(len(x1))]
        y1 = [y1[i]-yMass[i]*1000 for i in range(len(y1))]
        left = []
        for i in range(increment):
            left.extend(compute(x1[i],y1[i],z1[i],'r').transpose())
        left = np.transpose(left)
        left = left.tolist()
        
        return (right,left)
    
    
def computeTsup(stepLength):
    '''Your Code Here 1'''        ##########################
    return 2


def generateOutList(pace,stepLength): 
    global increment
    Tsup = computeTsup(stepLength) # compute Tsup given step5length
    timeList = [(float(j)+1)/increment*Tsup for j in range(increment*pace)]

    count = pace
    step = 0    # 0 for left, 1 for right
    leftLegList = []
    rightLegList = []
    
    for i in range(count):
        leg = legTrajectory(step,stepLength,Tsup)
        leftLegList = timeAppend(leftLegList,leg[0])
        rightLegList = timeAppend(rightLegList,leg[1])
        step = (step + 1) % 2
    return (leftLegList,rightLegList,timeList)
    
def move4(robotIP,pace,stepLength):

#--------------------------Initialize------------------------------------
    PORT = 9559
    try:
        motionProxy = ALProxy("ALMotion", robotIP, PORT)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)
    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e
    postureProxy.goToPosture("StandInit", 0.5)


#--------------------------First Pace -----------------------------------
    '''Your Code Here 2'''

#--------------------------Inverse Pendulum------------------------------
    #pace = 10  #input
    #stepLength = 0.1002 #input
    listof3 = generateOutList(pace,stepLength) 
    angleLists1 = listof3[0] 
    print len(angleLists1)
    angleLists1.extend(listof3[1])
    timeList = listof3[2]
    timeLists = []
    for i in range(12):
        timeLists.append(timeList)
    names1      = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",\
                 "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"]
    isAbsolute = True
    print "angleList length:  " + str(len(angleLists1))
    print "timeList length:  " + str(len(timeLists))
    motionProxy.angleInterpolation(names1, angleLists1, timeLists, isAbsolute)
#--------------------------Last Pace-------------------------------------
    '''Your Code Here 3'''
    #postureProxy.goToPosture("StandInit", 0.5)

    

if __name__ == "__main__":
    robotIp = "192.168.0.105"
    #robotIp = "127.0.0.1"    
    if len(sys.argv) <= 1:
        print "Usage python almotion_angleinterpolation.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]
    stepLength = 0.05
    pace = 8
    move4(robotIp,pace,stepLength)

