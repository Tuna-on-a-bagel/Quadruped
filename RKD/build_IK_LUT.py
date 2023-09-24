import numpy as np
from numpy import linalg as npl
import json
import kinematics
import math


'''File for solving 3D grid of IK solutions offline thta can be referenced online'''

def build_LUT(numPoints, configFile):

    '''
        Distributes grid uniformly along each limit axis

        Args:
            
            numPoints:      rough estimate number of grid coordinates to define within joint space, 
                            NOTE: usually not possible to get exact integers that will x*y*z==numPoints
            configeFile:    <path_to_config.json>   '''

    K = kinematics.Kinematics()                 #instantiate kineam class

    K.setConfig(configFile)                     #set DH params and joint limits

    conf = open(configFile, 'r')                #open config file
    data = json.load(conf)
    conf.close()
    legIDs = list(data['IK_solutions'].keys())  
    
    for idx, leg in enumerate(K.legs):

        grid = []

        lim_abad = leg.jointLimits[0]
        lim_shoulder = leg.jointLimits[1]
        lim_knee = leg.jointLimits[2]

        #determine length of each DOF to uniformly distribute grid points
        limLength_abad = np.abs(lim_abad[0] - lim_abad[1])
        limLength_shoulder = np.abs(lim_shoulder[0] - lim_shoulder[1])
        limLength_knee = np.abs(lim_knee[0] - lim_knee[1])

        #percentage splits
        p_abad = limLength_abad/(limLength_abad + limLength_shoulder + limLength_knee)
        p_shoulder = limLength_shoulder/(limLength_abad + limLength_shoulder + limLength_knee)
        p_knee = limLength_knee/(limLength_abad + limLength_shoulder + limLength_knee)

        #value when multiplied by percentages will == desired numPoints
        cubedNum = (numPoints/(p_abad*p_shoulder*p_knee))**(1/3)

        i = cubedNum*p_abad         #abad count
        j = cubedNum*p_shoulder     #shoulder count
        k = cubedNum*p_knee         #knee count

        '''
        #Note, there is a problem to face with rounding here, if we constantly round up, we could over shoot the
        # number of desired points by a signifigant amount (50-100%) if most of these floats are not near a round int, visa 
        # versa if we round down
        
        print(f'i, j, k: {i, j, k}')
        print('example to highlight rounding issue:')
        print(f'numPoints using floats = {i * j * k}')
        print(f'numPoints using ceil int values = {math.ceil(i) * math.ceil(j) * math.ceil(k)}')
        print(f'numPoints using floor int values = {math.floor(i) * math.floor(j) * math.floor(k)}')
        
        #nieve approach, round smallest number to ciel, round highest numbers to floor
        #print(f'nieve: {math.floor(i) * math.ceil(j) * math.floor(k)}')
        '''

        if i < j and i < k:
            i = math.ceil(i)
            j = math.floor(j)
            k = math.floor(k)

        elif j < i and j < k:
            i = math.floor(i)
            j = math.ceil(j)
            k = math.floor(k)

        else:
            i = math.floor(i)
            j = math.floor(j)
            k = math.ceil(k)

        #vector in each dimension based on percentage splits fromjoint limits
        abad_vals = np.linspace(lim_abad[0], lim_abad[1], i)
        shoulder_vals = np.linspace(lim_shoulder[0], lim_shoulder[1], j)
        knee_vals = np.linspace(lim_knee[0], lim_knee[1], k)

        angleVec = np.zeros((12, 1))

        for ith in range(i):
            for jth in range(j):
                for kth in range(k):

                    # update 12x1 angle vector for iter specific leg
                    angleVec[idx*3:idx*3+3] = np.array([abad_vals[ith],shoulder_vals[jth], knee_vals[kth]]).reshape(3, 1)

                    _, footPos = K.forwardK(angles=angleVec)        # compute x,y,z foot positions

                    [x, y, z] = footPos[idx*3:idx*3+3, 0]           # extract positions

                    grid.append([x, y, z, abad_vals[ith], shoulder_vals[jth], knee_vals[kth]])   # store solutions

        data['IK_solutions'][legIDs[idx]] = grid                    # store solutions

    conf = open(configFile, 'w')
    json.dump(data, conf, indent=4)
    conf.close()

if __name__ == '__main__':
    build_LUT(80, configFile='/home/tuna/Documents/Quad/RKD/roboConf.json')






    
