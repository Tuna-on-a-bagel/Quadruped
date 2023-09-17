import numpy as np
import json

import torch
from torch import matmul
from torch import norm
import autograd.numpy as np
from torch.autograd import grad
from torch.autograd.functional import jacobian
from torch.autograd.functional import hessian


class Leg():
    def __init__(self):
        self.jointPos = np.array([[0.], [0.], [0.]])    # 3x1 joint angle [ab/ad, shoulder, knee].T in base of leg frame       [rad]   [float]
        self.kneePos = np.array([[0.], [0.], [0.]])     # 3x1 [x, y, z].T in base of leg frame            [m]     [float]
        self.footPos = np.array([[0.], [0.], [0.]])     # 3x1 [x, y, z].T in base of leg frame            [m]     [float]

        self.DH = np.array([[0., 0., 0., 0.],           # DH parameters [a_i, alpha_i, d_i, theta_i]       [m, m, rad, rad] [float, float, float, float]
                            [0., 0., 0., 0.], 
                            [0., 0., 0., 0.]])
        
        self.jointLimits = np.array([[0., 0.], [0., 0.], [0., 0.]]) # 3x2 [[min, max (ab/ad)], [min, max (shoulder)], [min, max (knee)]]
        self.IK_solutions = []                                      # Binned solutions to IK problem for initial guess         
        
class Kinematics:

    def __init__(self):
        self.FL = Leg()
        self.BL = Leg()
        self.FR = Leg()
        self.BR = Leg()
        self.legs = [self.FL, self.BL, self.FR, self.BR]
        self.curFootPos = np.zeros((4, 3))   # 4x3 [[x1, y1, z1], [x2, y2, ...]]    [m]
        self.curKneePos = np.zeros((4, 3))   # 4x3
                              

    def __repr__(self, direction='forward', footPos=None, kneePos=None, angles=None):
        
        if footPos is not None and kneePos is not None:
          
            print(f'kneePos: \t footPos:')
            for i in range(12):
                print('%15s' % kneePos[i], '%15s' % footPos[i] )    

            return

    def setConfig(self, config_path):

        '''conf: <path_to_json_file> containing DenavitHartberg params as 'DH', joint limits as 'jointLimits
            
            NOTE: I am following DH convention via: https://github.com/users/Tuna-on-a-bagel/projects/3?pane=issue&itemId=33689584'''   

        file = open(config_path, 'r') 
        data = json.load(file)
        assert data is not None, 'No data was loaded from config file, check path integrity'

        #set DH params
        self.FL.DH[0][:] = data['DH']['FL'][0][:]
        self.FL.DH[1][:] = data['DH']['FL'][1][:]
        self.FL.DH[2][:] = data['DH']['FL'][2][:]
        self.BL.DH[0][:] = data['DH']['BL'][0][:]
        self.BL.DH[1][:] = data['DH']['BL'][1][:]
        self.BL.DH[2][:] = data['DH']['BL'][2][:]
        self.FR.DH[0][:] = data['DH']['FR'][0][:]
        self.FR.DH[1][:] = data['DH']['FR'][1][:]
        self.FR.DH[2][:] = data['DH']['FR'][2][:]
        self.BR.DH[0][:] = data['DH']['BR'][0][:]
        self.BR.DH[1][:] = data['DH']['BR'][1][:]
        self.BR.DH[2][:] = data['DH']['BR'][2][:]

        #set joint limits
        self.FL.jointLimits = data['jointLimits']['FL']
        self.BL.jointLimits = data['jointLimits']['BL']
        self.FR.jointLimits = data['jointLimits']['FR']
        self.BR.jointLimits = data['jointLimits']['BR']

        #set pre binned solutions
        self.FL.IK_solutions = data['IK_solutions']['FL']
        self.BL.IK_solutions = data['IK_solutions']['BL']
        self.FR.IK_solutions = data['IK_solutions']['FR']
        self.BR.IK_solutions = data['IK_solutions']['BR']

     
    def forwardK(self, angles=None, verbos=False):
        
        '''returns kneePos, footPos as two seperate 12x1 np.array([[x1], [y1], [z1], [x2], [y2] ...]) [m] 
           [IN LEG BASE FRAME, MUST TRANSFORM TO ROBOT FRAME, THEN TO WORLD IF NEEDED]

                if angles == None, will use current robot joint angels
                if angles == 12x1 (np.array),  will use provided angles (used to design traj) 
            
            transform = [[R x]          where R is 3x3 RotMat, x is 3x1 position vec, s is a stretch (usually 1)
                         [0 s]]'''

        kneePos = np.zeros((12, 1))
        footPos = np.zeros((12, 1))

        for idx, leg in enumerate(self.legs):

            #determine joint angles
            if angles == None: [theta0, theta1, theta2] = leg.jointPos[:, 0]
            
            else:
                assert angles.shape == (12, 1), f'invalid joint angle array shape, expected (12, 1), but {angles.shape} provided'
                [theta0, theta1, theta2] = angles[idx*3:(idx*3)+3, 0]
           
            #extract non variable params
            [a0, a1, a2] = leg.DH[:, 0]
            [d0, d1, d2] = leg.DH[:, 1]
            [alpha0, alpha1, alpha2] = leg.DH[:, 2]
            
            #precompute joint values
            c_th0, c_th1, c_th2 = np.cos(theta0), np.cos(theta1), np.cos(theta2)
            s_th0, s_th1, s_th2 = np.sin(theta0), np.sin(theta1), np.sin(theta2)
            c_al0, c_al1, c_al2 = np.cos(alpha0), np.cos(alpha1), np.cos(alpha2)
            s_al0, s_al1, s_al2 = np.sin(alpha0), np.sin(alpha1), np.sin(alpha2)

            #frame transforms
            base2Shoulder = np.array([[c_th0,   -s_th0*c_al0,   s_th0*s_al0,    a0*c_th0],
                                      [s_th0,   c_th0*c_al0,    -c_th0*s_al0,   a0*s_th0],
                                      [0.0,     s_al0,          c_al0,          d0],
                                      [0.0,     0.0,            0.0,            1.0]])
            
            shoulder2Knee = np.array([[c_th1,    -s_th1*c_al1,    s_th1*s_al1,    a1*c_th1],
                                      [s_th1,    c_th1*c_al1,     -c_th1*s_al1,   a1*s_th1],
                                      [0.0,      s_al1,           c_al1,          d1],
                                      [0.0,      0.0,             0.0,            1.0]])
            
            knee2foot = np.array([[c_th2,    -s_th2*c_al2,    s_th2*s_al2,    a2*c_th2],
                                  [s_th2,    c_th2*c_al2,     -c_th2*s_al2,   a2*s_th2],
                                  [0.0,      s_al2,           c_al2,          d2],
                                  [0.0,      0.0,             0.0,            1.0]])

            #compute intermediate frames
            base2Knee = base2Shoulder@shoulder2Knee 
            base2Foot = base2Knee@knee2foot

            #extract + insert [xi, yi, zi].T coords (upper right column vec)
            kneePos[idx*3:idx*3 + 3] = base2Knee[0:3, 3].reshape(3,1)
            footPos[idx*3:idx*3 + 3] = base2Foot[0:3, 3].reshape(3,1)
        
        if verbos: self.__repr__(footPos=footPos, kneePos=kneePos)

        return kneePos, footPos
    

    def inverseK(self, leg, footPos, maxIter=100, tol=1e-6, verbose=False):

        '''Args:
                leg:        Leg() class object
                footPos:    3x1 [x, y, z].T of leg base frame coordinates [np.array]
                maxIter:    max number of itterations allowable [int]
                tol:        norm tolerance for iter exit
                verbose:    if true will print out itteration history and convergence time
           
           output:
                returns 3x1 joint angles [th0, th1, th2].T if convergence met or max iter expires
                returns 'error: exceeded joint limits' if position is outside of c-space 
        '''
        
        # Im setting this up to do one leg at a time, I think trying to doo all the legs at once will come back to cause issues

        #TODO: bin IK solutions into roboConf.json

        IK_sols = leg.IK_solutions

        #check prebinned solutions for best starting guess 
        nearestSol = None
        nearestDist = 100      #arbitrary
        for sol in IK_sols:
            dist = np.linalg.norm(sol - footPos)        #float
            if dist < nearestDist:
                nearestSol = sol[3:, 1]                 # 3 x 1 array??? wanting this to be joint angles
                nearestDist = dist                      #float

        #TODO: make this a tensor
        x_ = nearestSol                                 #initial guess

        for iter in range(maxIter):

            cost = objective(x_)                        #forward pass
            cost.backward()                             #backward storing for backward mode diffing

            g = x_.grad                                 #gradient wrt x_

            #check for convergence:
            if norm(g) < tol:
                return x_, x_hist
            
            H = hessian(objective, x_)          #hessian_x(f(x))

            del_x = -torch.linalg.solve(H, g)   #solving Hx = g, x = -inv(H)/g

            #armijo's rule:
            a = 1.                              #step size multiplier
            b = 0.1                             #tolerance
            c = 0.4                             #scale factor
            
            while objective(x_ + a*del_x) > objective(x_) + b*a*g.t()@del_x:
                a *= c

            #update step size
            del_x #*= a

            #redefine tensor such that gradient sum is reset (pytorch-only neccessary)
            x_ = torch.tensor(x_ + del_x, requires_grad=True)

            #store step for visualization
            x_hist.append(x_)


            
        
            

        #Thoughts, it sounds like there will still be multiple solutions even in closed form. So might 
        # as well solve w Newton's method so I can penlalize illogical configurations, should measure wall clock cost
        # of convergence though.
        #
        # Alsp, I know basically that the ab/ad DOF will be within a 60* window from leg base frame, and similar assumptions can be made about other joints. I can
        # use this to my advantage when picking an initial guess. Honestly I think this is a simple enough problem that I could just bin a few solutions to grid point
        # locations and use those as my starting initial guess. for example I could solve the problem by trial and error to get solutions for -20:2:20 for x, etc for
        # y and z, then just check where the nearest solution is before I start newtons method. I think I could generally solve in this manner with 5-6 itterations max.

        return 
    
''''
def objective(x_):

    l1 = l[0]
    l2 = l[1]
    l3 = l[2]

    theta1 = x_[0]
    theta2 = x_[1]
    theta3 = x_[2]

    x_base = l1*torch.cos(theta1) + l2*torch.cos(theta1 + theta2) + l3*torch.cos(theta1 + theta2 + theta3)
    y_base = l1*torch.sin(theta1) + l2*torch.sin(theta1 + theta2) + l3*torch.sin(theta1 + theta2 + theta3)
    
    x_world = x_base + robot.baseX
    y_world = y_base + robot.baseY

    out = 0.5*((x_world - app.goal[0])**2 + (y_world - app.goal[1])**2)
    #out = 0.5*norm(x_world - app.goal[0] + y_world - app.goal[1])**2
    return out
    #stuff that didnt work:
    #xy = torch.tensor([x_world, y_world], requires_grad=True)
    #out = torch.tensor([x_world - app.goal[0], y_world - app.goal[1]], requires_grad=True)
    #out = torch.tensor([x_base - app.goal[0], y_base - app.goal[1]], requires_grad=True)
    #out = 0.5*(torch.norm(xy - app.goal))**2
    
   


def solveIK(robot, app, tol):

    maxiter = 50

    theta1 = copy.copy(robot.states[0])
    theta2 = copy.copy(robot.states[2])
    theta3 = copy.copy(robot.states[4])

    #state vector
    x_ = torch.tensor([theta1, theta2, theta3], requires_grad=True)

    print(f'init:    x_: [{torch.round(x_[0], decimals=3)},{torch.round(x_[1], decimals=3)}, {torch.round(x_[1], decimals=3)}]')

    #storing newton steps for visulizing the algorithm
    x_hist = []
    
    for iter in range(maxiter):

        cost = objective(x_)                #forward pass
        cost.backward()                     #backward storing for backward mode diffing

        g = x_.grad                         #gradient wrt x_

        #check for convergence:
        if norm(g) < tol:
            return x_, x_hist
        
        H = hessian(objective, x_)          #hessian_x(f(x))

        del_x = -torch.linalg.solve(H, g)   #solving Hx = g, x = -inv(H)/g

        #armijo's rule:
        a = 1.                              #step size multiplier
        b = 0.1                             #tolerance
        c = 0.4                             #scale factor
        
        while objective(x_ + a*del_x) > objective(x_) + b*a*g.t()@del_x:
            a *= c

        #update step size
        del_x #*= a

        #redefine tensor such that gradient sum is reset (pytorch-only neccessary)
        x_ = torch.tensor(x_ + del_x, requires_grad=True)

        #store step for visualization
        x_hist.append(x_)


        #print(f'iter: {iter}, x_: [{torch.round(x_[0], decimals=3)}, {torch.round(x_[1], decimals=3)}, {torch.round(x_[2], decimals=3)}], cost: {cost}')

    print('-------')
        
    return x_, x_hist'''
    

if __name__ == '__main__':

    K = Kinematics()
    K.setConfig('/home/tuna/Documents/Quad/roboConf.json')
    K.forwardK(verbos=True)

    K.inverseK()
