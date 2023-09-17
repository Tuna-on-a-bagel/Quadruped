import numpy as np
from numpy import linalg as npl
import json
import kinematics
import math


'''File for solving 3D grid of IK solutions offline thta can be referenced online'''

def buildGrid(numPoints, configFile):

    '''
        Distributes grid uniformly along each limit axis

        Args:
            
            numPoints:      rough estimate number of grid coordinates to define within joint space, 
                            NOTE: usually not possible to get exact integers that will x*y*z==numPoints
            configeFile:    <path_to_config.json>   '''

    K = kinematics.Kinematics()

    K.setConfig(configFile)

    #TODO: loopThrough forward K

    conf = open(configFile, 'r')
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
        #print(f'limLengths: {limLength_abad, limLength_shoulder, limLength_knee}')

        #input vec magnitude
        n_mag = np.sqrt(limLength_abad**2 + limLength_shoulder**2 + limLength_knee**2)
        #print(f'n_mag: {n_mag}')

        #input components
        n_abad =  limLength_abad / n_mag
        n_shoulder = limLength_shoulder / n_mag 
        n_knee =  limLength_knee / n_mag

        #unit
        u_mag = np.sqrt(n_abad**2 + n_shoulder**2 + n_knee**2)
       
        u_abad = n_abad / u_mag
        u_shoulder = n_shoulder / u_mag
        u_knee = n_knee / u_mag
        u = u_abad + u_shoulder + u_knee

        #percentage splits
        p_abad = u_abad/u
        p_shoulder = u_shoulder/u
        p_knee = u_knee/u

        #value when multiplied by percentages will == desired numPoints
        cubedNum = (numPoints/(p_abad*p_shoulder*p_knee))**(1/3)

        i = cubedNum*p_abad         #abad count
        j = cubedNum*p_shoulder     #shoulder count
        k = cubedNum*p_knee         #knee count

        #Note, there is a problem to face with rounding here, if we constantly round up, we could over shoot the
        # number of desired points by a signifigant ammount if most of these floats are not near a round int, visa 
        # versa if we round down
        '''
        print(f'i, j, k: {i, j, k}')
        print('example to highlight rounding issue:')
        print(f'numPoints using floats = {i * j * k}')
        print(f'numPoints using ceil int values = {math.ceil(i) * math.ceil(j) * math.ceil(k)}')
        print(f'numPoints using floor int values = {math.floor(i) * math.floor(j) * math.floor(k)}')
        '''
        #nieve approach, round smallest number to ciel, round highest numbers to floor
        #print(f'nieve: {math.floor(i) * math.ceil(j) * math.floor(k)}')

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
        
        #print(f'nieve: {i*j*k}')

        abad_vals = np.linspace(lim_abad[0], lim_abad[1], i)
        shoulder_vals = np.linspace(lim_shoulder[0], lim_shoulder[1], j)
        knee_vals = np.linspace(lim_knee[0], lim_knee[1], k)

        angleVec = np.zeros((12, 1))
        for ith in range(i):
            for jth in range(j):
                for kth in range(k):
                    #TODO: add matching foot positions 1x3 using forwardK
                    
                    angleVec[idx*3:idx*3+3] = np.array([abad_vals[ith],shoulder_vals[jth], knee_vals[kth]]).reshape(3, 1)
                    #print(angleVec)
                    _, footPos = K.forwardK(angles=angleVec)

                    [x, y, z] = footPos[idx*3:idx*3+3, 0]
                    #print(f'xyz: {x, y, z}')

                    grid.append([x, y, z, abad_vals[ith],shoulder_vals[jth], knee_vals[kth]])

        #store solutions
        data['IK_solutions'][legIDs[idx]] = grid
        
        
        
        grid = np.array(grid)
        print(grid.shape)

    conf = open(configFile, 'w')
    json.dump(data, conf, indent=4)
    conf.close()

if __name__ == '__main__':
    buildGrid(80,configFile='/home/tuna/Documents/Quad/RKD/roboConf.json')






    
