#import numpy as np

import pygame
import pygame.locals
import copy

import torch
from torch import matmul
from torch import norm
import autograd.numpy as np
from torch.autograd import grad
from torch.autograd.functional import jacobian
from torch.autograd.functional import hessian

'''

Working inverse kinematics solver for planarBot. Newton-raphson with armijo backtracking line search.

'''




'''
robot state vector = [l1.theta
                      l1.thetaDot
                      l2.theta
                      ...
                      ]'''

class App():

    def __init__(self):
        pygame.init()
 
        self.width = 750
        self.height = 750

        self.start = (200, 180)
        self.goal = None

        self.fpsClock = pygame.time.Clock()
        self.display = pygame.display.set_mode((self.width, self.height))


class Robot():

    def __init__(self):
        self.base = None    #make this a link
        self.baseX = 0
        self.baseY = 0
        self.end = None     #make this final link
        self.links = []     #add links here
        self.states = np.array([])

    def addLink(self, link):

        if len(self.links) == 0:
            link.baseX = copy.copy(self.baseX)
            link.baseY = copy.copy(self.baseY)

        self.links.append(link)
        self.states = np.append(self.states, np.array([link.theta, link.thetaDot]))

    def updateKinematics(self):

        for i in range(0, len(self.links)):
            link = self.links[i]
            link.theta = copy.copy(self.states[2*i]) #actual relative angle
            link.displayTheta = sum(self.states[2*j] for j in range(i)) + copy.copy(self.states[2*i]) #angle for painting robot
            link.thetaDot = copy.copy(self.states[2*i + 1])

        for i in range(1, len(self.links)):
            self.links[i].baseX = self.links[i-1].baseX + self.links[i-1].getRelativeEndPoint()[0]
            self.links[i].baseY = self.links[i-1].baseY + self.links[i-1].getRelativeEndPoint()[1]

    def solveForwardK(self):

        l1 = self.links[0]
        l2 = self.links[1]
        l3 = self.links[2]   

        x_base = l1.length*np.cos(l1.theta) + l2.length*np.cos(l1.theta + l2.theta) + l3.length*np.cos(l1.theta + l2.theta + l3.theta)
        y_base = l1.length*np.sin(l1.theta) + l2.length*np.sin(l1.theta + l2.theta) + l3.length*np.sin(l1.theta + l2.theta + l3.theta)

        out = torch.tensor([x_base, y_base]) #robot base frame
        outWorld = torch.tensor([x_base + self.baseX, y_base + self.baseY], requires_grad=True) #robot world frame
        #return (x_world, y_world)
        return out, outWorld
    
    def solveForwardK_explicit(self, x_):

        l1 = self.links[0]
        l2 = self.links[1]
        l3 = self.links[2]  

        theta1 = x_[0].item()
        theta2 = x_[1].item()
        theta3 = x_[2].item()

        x_base = l1.length*np.cos(theta1) + l2.length*np.cos(theta1 + theta2) + l3.length*np.cos(theta1 + theta2 + theta3)
        y_base = l1.length*np.sin(theta1) + l2.length*np.sin(theta1 + theta2) + l3.length*np.sin(theta1 + theta2 + theta3)

        out = torch.tensor([x_base, y_base]) #in robot base frame
        outWorld = torch.tensor([x_base + self.baseX, y_base + self.baseY], requires_grad=True) #in robot world frame
        #return (x_world, y_world)
        return out, outWorld

    

class Link():

    def __init__(self):
        self.length = 0
        self.mass = 0
        self.theta = 0
        self.thetaDot = 0
        self.displayTheta = 0
        self.baseX = 0
        self.baseY = 0
        self.child = None
        self.parent = None


    def setImage(self):
        self.pyLink = pygame.Surface((self.length, 15), pygame.SRCALPHA, 32)
        self.pyLink.fill((50, 50, 50, 200))

    def getRelativeEndPoint(self):

        endX = self.length * np.cos(self.displayTheta)
        endY = self.length * np.sin(self.displayTheta)

        return [endX, endY]



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
        
    return x_, x_hist
        




def drawRobot(app):

    for num, link in enumerate(robot.links):

            #draw links
            pygame.draw.line(surface=app.display, 
                            color=(100, 100, 100), 
                            start_pos=(link.baseX,link.baseY), 
                            end_pos=(link.baseX + link.getRelativeEndPoint()[0], 
                                    link.baseY + link.getRelativeEndPoint()[1]),
                            width = 5)
            
            #draw 'joint' positions
            pygame.draw.circle(surface=app.display,
                            center=(link.baseX, link.baseY),
                            color=(0, 0, 0),
                            radius= 6,
                            width=15)
            

def main(app):

    limit = 0
    #detirmine c-space radius (assumes no joint limits)
    for link in robot.links:
        limit += link.length

    #detirmine robot end affector init position
    robot.updateKinematics()
    forwardK, forwardK_world = robot.solveForwardK()

    #positions of interest
    start_x, start_y = copy.copy(forwardK_world[0].item()), copy.copy(forwardK_world[1].item())
    goal_x, goal_y = copy.copy(app.goal[0].item()), copy.copy(app.goal[1].item())


    #NOTE: this is just a place holder loop to delay the start so you can familiarize with problem
    #init display
    screenCap = True
    for i in range(30):
        app.display.fill((255, 255, 255))
        #drawing c-space
        pygame.draw.circle(surface=app.display,
                           color=(230, 230, 230),
                           center=(robot.baseX, robot.baseY),
                           radius=limit
        )
        
        #goal point
        pygame.draw.circle(surface=app.display,
                           color= (0, 200, 0),
                           center=(app.goal[0].item(), app.goal[1].item()),  #.item turns a tensor element into a python number
                           radius=5)
        #print(robot.states)
        pygame.draw.polygon(surface=app.display,
                            color=(0,0,0),
                            points=[(robot.baseX - 20, robot.baseY + 30),
                                    (robot.baseX, robot.baseY),
                                    (robot.baseX + 20, robot.baseY + 30)])
        
        drawRobot(app)

        #draw predicted end effector position
        pygame.draw.circle(surface=app.display,
                           color= (200, 0, 200),
                           center=(forwardK_world[0].item(), forwardK_world[1].item()),
                           radius=5)
        
        #draw labels
        font = pygame.font.Font('freesansbold.ttf', 15)
        textS = font.render('Start', True, (0, 0, 0), None)
        textG = font.render('Goal', True, (0, 0, 0), None)
        textIter = font.render(f'iter: {0}', True, (0, 0, 0), None)
        textRS = textS.get_rect()
        textRG = textG.get_rect()
        textRIter = textIter.get_rect()
        textRS.center = (start_x, start_y - 15)
        textRG.center = (goal_x, goal_y - 15)
        textRIter.center = (robot.baseX, robot.baseY - 310)
        app.display.blit(textS, textRS)
        app.display.blit(textG, textRG)
        app.display.blit(textIter, textRIter)
        
        pygame.display.update()
        if screenCap:
            #pygame.image.save(app.display, f"/home/tuna/Documents/driving/control/planarBot/NRsingularity_gif/singularNR{0}.png")
            screenCap = False

        app.fpsClock.tick(30)

    screenCap = True
    #solve optimization problem, return solved joint angles (x) and newton step itterations (x_hist)
    x, x_hist = solveIK(robot, app, 1e-4)
    
    i = 0
    while True:

        #show newton steps
        if i < len(x_hist):
            x = x_hist[i]                   #extract each state tensor from history
            robot.states[0] = x[0].item()   #update theta1
            robot.states[2] = x[1].item()   #update theta2
            robot.states[4] = x[2].item()   #update theta3

            #update visualization variables
            robot.updateKinematics()

            #solve for placement of expected end affector position
            forwardK, forwardK_world = robot.solveForwardK()

            i += 1

        #clear old display
        app.display.fill((255, 255, 255))

        #draw reachable c-space (NOTE: reaching any point on the limit will result in singularity)
        pygame.draw.circle(surface=app.display,
                           color=(240, 240, 240),
                           center=(robot.baseX, robot.baseY),
                           radius=limit
                           )
        

        #print(robot.states)
        pygame.draw.polygon(surface=app.display,
                            color=(0,0,0),
                            points=[(robot.baseX - 20, robot.baseY + 30),
                                    (robot.baseX, robot.baseY),
                                    (robot.baseX + 20, robot.baseY + 30)])
        
    
        drawRobot(app)
        
        #draw predicted end effector position
        pygame.draw.circle(surface=app.display,
                           color= (200, 0, 200),
                           center=(forwardK_world[0].item(), forwardK_world[1].item()),
                           radius=5)

        #start point
        pygame.draw.circle(surface=app.display,
                           color= (200, 0, 0),
                           center=(start_x, start_y),  #.item turns a tensor element into a python number
                           radius=5)
        
        #goal point
        pygame.draw.circle(surface=app.display,
                           color= (0, 200, 0),
                           center=(app.goal[0].item(), app.goal[1].item()),  #.item turns a tensor element into a python number
                           radius=5)

        #draw labels
        font = pygame.font.Font('freesansbold.ttf', 15)
        textS = font.render('Start', True, (0, 0, 0), None)
        textG = font.render('Goal', True, (0, 0, 0), None)
        textIter = font.render(f'iter: {i}', True, (0, 0, 0), None)
        textRS = textS.get_rect()
        textRG = textG.get_rect()
        textRIter = textIter.get_rect()
        textRS.center = (start_x, start_y - 15)
        textRG.center = (goal_x, goal_y - 15)
        textRIter.center = (robot.baseX, robot.baseY - 310)
        app.display.blit(textS, textRS)
        app.display.blit(textG, textRG)
        app.display.blit(textIter, textRIter)

        
        pygame.display.update()
        if screenCap and i < len(x_hist):
            #pygame.image.save(app.display, f"/home/tuna/Documents/driving/control/planarBot/NRsingularity_gif/singularNR{i+1}.png")
            None

        app.fpsClock.tick(3)
        # check for quit
        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT:
                pygame.quit()
            
if __name__ == "__main__":

    app = App()

   

    robot = Robot()
    robot.baseX = app.width/2
    robot.baseY = app.height/2

    l1 = Link()
    l1.length = 100.
    l1.theta = -np.pi/12

    l2 = Link()
    l2.length = 100.
    l2.theta = -np.pi/2

    l3 = Link()
    l3.length = 100.
    l3.theta = np.pi/4

    #l4 = Link()
    #l4.length = 40

    robot.addLink(l1)
    robot.addLink(l2)
    robot.addLink(l3)
    #robot.addLink(l4)

    #app.goal = torch.tensor([robot.baseX - 50, robot.baseY - 270.], requires_grad=True)
    app.goal = torch.tensor([robot.baseX - 250, robot.baseY - 80], requires_grad=True)

    l = torch.tensor([robot.links[0].length, robot.links[1].length, robot.links[2].length], dtype=torch.float64)
    main(app)


'''

build a cost function that penalizes joint angles that do not satid=sfay end effector position

start with 2 joints:

min_x

'''