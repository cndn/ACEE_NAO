import numpy as np
import math
from scipy.linalg import *
from forwardKinematics import *
import random
def jacobianInverse(target,lorr,fix = [0,0,0,0,0,0]):
    alpha = 0.8
    clamp = 10
    maxiter = 50
    np.set_printoptions(precision=3)
    np.set_printoptions(suppress=True)
    #mask = np.ones((4,4))
    initial = np.zeros((6,1))
    
    (T,D) = fLeg(initial,lorr)
    J = np.zeros((6,16))
    tol = 0.001
    quality = sum(sum(target - T))
    iteration = 0
    while (abs(quality) > tol and iteration <= maxiter):
        iteration += 1
        (T,D) = fLeg(initial,lorr)
        quality = sum(sum(target - T))
        e = target - T
        #if iteration == 1:
        #    print e
        m = np.max(np.max(abs(e)))
        if m > clamp :
            e = e/m*clamp
            
        for i in range(6):
            p = np.transpose(D[i]).reshape(16)
            #if iteration == 1:
            #    print D[i]
            #    print p
            J[i] = p
            
       # if iteration == 1:
       #     print J
       #     print pinv(np.transpose(J))
        e = np.transpose(e).reshape((16,1))
        g = np.dot(alpha*pinv(np.transpose(J)),e)
        initial += g
        #if iteration == 1:
        #    print e
        if lorr == 'r':
            if initial[0] > 0.74 or initial[0] < -1.145:
                initial[0] = random.random()
            if initial[1] > 0.414 or initial[1] < -0.738:
                initial[1] = random.random()
            if initial[2] > 0.486 or initial[2] < -1.772:
                initial[2] = random.random()
            if initial[3] > 2.12 or initial[3] < -0.103:
                initial[3] = random.random()
            if initial[4] > 0.93 or initial[4] < -1.18:
                initial[4] = random.random()
            if initial[5] > 0.387 or initial[5] < -0.786:
                initial[5] = random.random()
            for i in range(6):
                if fix[i] != 0:
                    if abs(initial[i] - fix[i]) > 0.02:
                        initial[i] = fix[i]
        elif lorr == 'l':
            if initial[0] > 0.74 or initial[0] < -1.145:
                initial[0] = random.random()
            if initial[1] > 0.79 or initial[1] < -0.379:
                initial[1] = random.random()
            if initial[2] > 0.484 or initial[2] < -1.772:
                initial[2] = random.random()
            if initial[3] > 2.112 or initial[3] < -0.092:
                initial[3] = random.random()
            if initial[4] > 0.923 or initial[4] < -1.189:
                initial[4] = random.random()
            if initial[5] > 0.769 or initial[5] < -0.398:
                initial[5] = random.random()
            for i in range(6):
                if fix[i] != 0:
                    if abs(initial[i] - fix[i]) > 0.02:
                        initial[i] = fix[i]
    thetas = initial
    return thetas
if __name__ == '__main__':
    target = fLeg([0,0,-.28,.432,-.224,0],'l')[0]
    print target
    jacobianInverse(target,'l')
    
