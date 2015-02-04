from DH import *
from ROT import *
import numpy as np
import math
from math import pi

global shoulderOffsetY
global elbowOffsetY
global upperArmLength
global shoulderOffsetZ
global LowerArmLength
global HandOffsetX
global HandOffsetZ
global HipOffsetZ
global HipOffsetY
global ThighLength
global TibiaLength
global FootHeight
global NeckOffsetZ

shoulderOffsetY = 98;
elbowOffsetY = 15;
upperArmLength = 105;
shoulderOffsetZ = 100;
HandOffsetX = 57.75;
HandOffsetZ = 12.31;
LowerArmLength = 55.95;
HipOffsetZ = 85; 
HipOffsetY = 50;
ThighLength = 100; 
TibiaLength = 102.90;
FootHeight = 45.11;
NeckOffsetZ = 126.5;

def fLeg(thetas,lorr): #HipYawPitch HipRoll HipPitch KneePitch AnklePitch AnkleRoll T1-T6
                       #unit CM
    base = np.eye(4);
    if lorr == 'r':
        base[1][3] = -HipOffsetY
        T1 = DH(0,-pi/4,0,thetas[0]-pi/2)
        dT1 = DHDerivative(0,-pi/4,0,thetas[0]-pi/2)
        T2 = DH(0,-pi/2,0,thetas[1]-pi/4) #+- left right diff?
        dT2 = DHDerivative(0,-pi/2,0,thetas[1]-pi/4)
    elif lorr == 'l':
        base[1][3] = HipOffsetY
        T1 = DH(0,-3*pi/4,0,thetas[0]-pi/2)
        dT1 = DHDerivative(0,-3*pi/4,0,thetas[0]-pi/2)
        T2 = DH(0,-pi/2,0,thetas[1]+pi/4) #+- left right diff?
        dT2 = DHDerivative(0,-pi/2,0,thetas[1]+pi/4)
    base[2][3] = -HipOffsetZ
    
    T3 = DH(0,pi/2,0,thetas[2])
    dT3 = DHDerivative(0,pi/2,0,thetas[2])
    T4 = DH(-ThighLength,0,0,thetas[3])
    dT4 = DHDerivative(-ThighLength,0,0,thetas[3])
    T5 = DH(-TibiaLength,0,0,thetas[4])
    dT5 = DHDerivative(-TibiaLength,0,0,thetas[4])
    T6 = DH(0,-pi/2,0,thetas[5])
    dT6 = DHDerivative(0,-pi/2,0,thetas[5])
    Tend = rotZYX(pi,-pi/2,0) #??
    Tend1 = np.eye(4)
    Tend1[2][3] = -FootHeight
    Tendend = reduce(np.dot,[base,T1,T2,T3,T4,T5,T6,Tend,Tend1])

    Derivatives1 = reduce(np.dot,[base,dT1,T2,T3,T4,T5,T6,Tend,Tend1])
    Derivatives2 = reduce(np.dot,[base,T1,dT2,T3,T4,T5,T6,Tend,Tend1])
    Derivatives3 = reduce(np.dot,[base,T1,T2,dT3,T4,T5,T6,Tend,Tend1])
    Derivatives4 = reduce(np.dot,[base,T1,T2,T3,dT4,T5,T6,Tend,Tend1])
    Derivatives5 = reduce(np.dot,[base,T1,T2,T3,T4,dT5,T6,Tend,Tend1])
    Derivatives6 = reduce(np.dot,[base,T1,T2,T3,T4,T5,dT6,Tend,Tend1])
    Derivatives = np.array([Derivatives1,Derivatives2,Derivatives3,Derivatives4,Derivatives5,Derivatives6])

    rotZ = math.atan2(Tendend[1][0],Tendend[0][0])
    rotY = math.atan2(-Tendend[2][0],math.sqrt(Tendend[2][1]**2 + Tendend[2][2]**2))
    rotX = math.atan2(Tendend[2][1],Tendend[2][2])
    #print Tendend
    #print (rotX,rotY,rotZ)
    return (Tendend,Derivatives)

if __name__ == '__main__':
    fLeg([0,0,-.28,.432,-.224,0],'l')
