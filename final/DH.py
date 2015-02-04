import numpy as np
import math

def DH(a,alpha,d,theta):
    
    DH = [[math.cos(theta),-math.sin(theta),0,a],\
          [math.sin(theta)*math.cos(alpha),math.cos(theta)*math.cos(alpha),-math.sin(alpha),-math.sin(alpha)*d],\
          [math.sin(theta)*math.sin(alpha),math.cos(theta)*math.sin(alpha),math.cos(alpha),math.cos(alpha)*d],\
          [0,0,0,1]]
    
    DH = np.array(DH)
    
    return DH
          
def DHDerivative(a,alpha,d,theta):
    
    DHDerivative = [[-math.sin(theta),-math.cos(theta),0,0],\
          [math.cos(theta)*math.cos(alpha),-math.sin(theta)*math.cos(alpha),0,0],\
          [math.cos(theta)*math.sin(alpha),-math.sin(theta)*math.sin(alpha),0,0],\
          [0,0,0,0]]
    
    DHDerivative = np.array(DHDerivative)
    
    return DHDerivative
