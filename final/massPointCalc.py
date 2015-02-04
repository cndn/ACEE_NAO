# -*- encoding: UTF-8 -*-
import math
import numpy as np
from scipy.integrate import *

Tc = 0.179
def endvx(Tsup,x):
    global Tc
    vx = x*(math.cosh(Tsup/Tc)+1)/(Tc*math.sinh(Tsup/Tc))
    return vx

def endvy(Tsup,y):
    global Tc
    vy = y*(math.cosh(Tsup/Tc)-1)/(Tc*math.sinh(Tsup/Tc))
    return vy

def testx(beginx,px,Tsup,t):
    beginvx = endvx(Tsup,(px-beginx))
    return (beginx-px)*math.cosh(t/Tc)+Tc*beginvx*math.sinh(t/Tc)+px

def xsolution(beginx,px,Tsup,timeList):
    global Tc
    beginvx = endvx(Tsup,px-beginx)
    print beginvx
    to = Tc*math.log((beginx-Tc*beginvx)/(px-Tc*beginvx))
    #return [(beginx-px)*math.cosh(t/Tc)+Tc*beginvx*math.sinh(t/Tc)+px for t in timeList] 
    return [ 0.0125*t/Tsup for t in timeList]

def vyApproximate(t,y,Tsup):
    beginvy = -endvy(Tsup,y)
    vy = beginvy-t/Tsup*2*beginvy
    return vy

def ysolution(step,Tsup,timeList):#y??
    global Tc
    y = 0.05
    beginvy = -endvy(Tsup,y)
    
    #ylist = [y*math.cosh(t/Tc)+Tc*beginvy*math.sinh(t/Tc) for t in timeList] #/2?
    ylist = [0.08*math.sin(t*157/Tsup) for t in timeList]
    #if step == 0:
    #   ylist = [-y*math.cosh(t/Tc)-Tc*beginvy*math.sinh(t/Tc) for t in timeList]
    #return ylist
    return  ylist
    
def computeTrajectory(timeList,Tsup,beginx,endx,beginvx,endvx):
    pass

def explode(x,v,p):
    while(1):
        nextx = 1.692*x + .3028*v -.692*p
        nextv = 9.452*x + 1.692*v -7.626*p
        print (nextx,nextv)
        x = nextx
        v = nextv
    
if __name__ =='__main__':
    Tsup = 4
    increment = 10
    timeList = [(float(j)+1)/increment*Tsup for j in range(increment)]
    print timeList
    print xsolution(-0.1,0.1,Tsup,timeList)
    print ysolution(0,Tsup,timeList)
    #print xsolution(0,0.05,Tsup,[(float(j)+1)/increment*Tsup for j in range(increment)])
    #explode(0.1,0.1,0)
