import numpy as np
import math

def rotXYZ(x,y,z): #input angle
    np.set_printoptions(threshold='nan')
    Rx = [[1,0,0],\
          [0,math.cos(x),-math.sin(x)],\
          [0,math.sin(x),math.cos(x)]]
    Rx = np.array(Rx)
                  
    Ry = [[math.cos(y),0,math.sin(y)],\
          [0,1,0],\
          [-math.sin(y),0,math.cos(y)]]
    Ry = np.array(Ry)

    Rz = [[math.cos(z),-math.sin(z),0],\
          [math.sin(z),math.cos(z),0],\
          [0,0,1]]
    Rz = np.array(Rz)

    R = np.concatenate((reduce(np.dot, [Rx,Ry,Rz]),np.zeros((3,1))),axis = 1)
    R = np.concatenate((R,np.array([[0,0,0,1]])),axis = 0)
    return R

def rotZYX(z,y,x):
    np.set_printoptions(threshold='nan')
    Rx = [[1,0,0],\
          [0,math.cos(x),-math.sin(x)],\
          [0,math.sin(x),math.cos(x)]]
    Rx = np.array(Rx)
                  
    Ry = [[math.cos(y),0,math.sin(y)],\
          [0,1,0],\
          [-math.sin(y),0,math.cos(y)]]
    Ry = np.array(Ry)

    Rz = [[math.cos(z),-math.sin(z),0],\
          [math.sin(z),math.cos(z),0],\
          [0,0,1]]
    Rz = np.array(Rz)

    R = np.concatenate((reduce(np.dot, [Rz,Ry,Rx]),np.zeros((3,1))),axis = 1)
    R = np.concatenate((R,np.array([[0,0,0,1]])),axis = 0)
    return R
