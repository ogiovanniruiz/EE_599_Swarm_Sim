import pygame

from pygame.locals import *

import numpy as np

import math

import matplotlib.pyplot as plt

import random

import time

import csv

red = [255, 0, 0]
green = [0, 255, 0]
blue = [0, 0, 255]
white = [255, 255, 255]
black = [0, 0, 0]

SCREENSIZE = [800, 800]  # Size of our output display

running = True
connections = True

N = 8 #Number of Robots
O = N #number of obstacles
bot_radius = 10
obj_radius = 10
Dim = 2  #Dimension of search space
count = 0

W = np.zeros((N, N), dtype=np.float) # The Swarm Weights Matrix
V = np.zeros((N, N), dtype=np.float) # The Swarm Energy Matrix
A = np.zeros((N, N), dtype=np.float) # The Swarm Adjacency Matrix

w_obj = np.zeros((N,O), dtype=np.float)
v_obj = np.zeros((N,O), dtype=np.float)
v_target = np.zeros((N), dtype=np.float)

K = 500

K_obj = 500

E_converge = []
E_static = []
E_dynamic = []

t_converge = []
t_static = []
t_dynamic = []

converge = open ('converge_dist.csv', 'a')
writer_converge = csv.writer(converge)

static = open ('static.csv', 'a')
writer_static = csv.writer(static)

dynamic = open ('dynamic.csv', 'a')
writer_dynamic = csv.writer(dynamic)

class Swarm_Simulation:
    def __init__(self):

        pygame.init()
        self.screen = pygame.display.set_mode(SCREENSIZE)
        self.screen.fill(white)
        pygame.display.set_caption("Distributed Simulation")

        self.myfont = pygame.font.SysFont("arial", 25)

        self.O = SCREENSIZE[0] * np.random.rand(Dim, O)# Obstacle Positions

        self.P = SCREENSIZE[1] * np.random.rand(Dim, N)  # Initial positions of robots in the search space.
        self.Pn = np.zeros((Dim, N), dtype=np.int) # The updated postions are initialized.

        self.R = np.random.rand(N)  # Initial rotation of robots in 2 pi radians.
        self.Rn = np.zeros((N), dtype=np.float) # The updated rotations are initialized.
        
        self.phi = np.zeros((N,N), dtype=np.float) #Input angles to robot controllers.
        self.phi_obj = np.zeros((N,O), dtype=np.float) #Input angles to robot controllers.
        self.phi_target = np.zeros((N), dtype=np.float)

        self.U_target = np.zeros((Dim, N), dtype=np.int) #Input desired movement direction for controllers.
        self.U_obj = np.zeros((Dim, N), dtype=np.int)
        self.U_bot = np.zeros((Dim, N), dtype=np.int)
        
        self.s = np.zeros((N), dtype=np.float) #Desired speeds of each robot.

        self.min = 50 #The target inter-robot distances.

        self.V_prev = 0 #The energy of the system.

        self.delta_V = 0 # Change in energy of the system in one program loop.

        self.b = 1.0 #A counter to reduce spacings for non-connected robots.

        self.a = 1.0 #Spacer multiplier.

        self.counter = 0 #Counter increases whenever the energy begins to increases.

        self.dynamic = np.zeros((N), dtype=np.float)

        self.dynamic_obstacles = False

        self.obstacle_speeds = 0

        self.dist_0 = np.zeros((N), dtype=np.float)

        self.minimum = 0

        self.move = 0

        self.target = [0,0]

        self.state = 0

        self.num_collisions = 0

        self.time_0 = time.time()

        self.time_1 = 0

        self.time_2 = 0

    def Run(self):

        while(running):

            self.screen.fill(white)

            #The intial energy of the system.
            self.V_prev = 0.5*V.sum()

            #The positions and orientations of all bots are updated.
            self.Pn = self.P 
            self.Rn = self.R

            sim.bot_weights()

            sim.test_cases()

            sim.obj_weights()

            for i in range(N):
                for j in range(N):

                        self.phi_target[i] = math.atan2(self.target[1] - self.P[1,i] , self.target[0] - self.P[0,i]) - self.Rn[i]

                        self.U_target[:,i] = [np.linalg.norm(self.target[:] - self.Pn[:, i] ) * math.cos(self.phi_target[i])*v_target[i],
                                              np.linalg.norm(self.target[:] - self.Pn[:, i] ) * math.sin(self.phi_target[i])*v_target[i]]

                        #===========================================================================================================

                        self.phi_obj[i,j] = math.atan2(self.O[1,j] - self.P[1,i] , self.O[0,j] - self.P[0,i]) - self.Rn[i]

                        self.U_obj[:,i] = [np.linalg.norm(self.O[:, j] - self.Pn[:, i] ) * math.cos(self.phi_obj[i,j]) * w_obj[i,j],
                                           np.linalg.norm(self.O[:, j] - self.Pn[:, i] ) * math.sin(self.phi_obj[i,j]) * w_obj[i,j]]

                        #=============================================================================================================

                        # Inter Robot i's angle input
                        self.phi[i,j] = math.atan2(self.P[1,j] - self.P[1,i], self.P[0,j] - self.P[0,i]) - self.Rn[i] 
                        self.phi[i,i] = 0
                
                        # Inter robot Rotation is calculated
                        self.U_bot[:, i] = [np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) * math.cos(self.phi[i,j])*W[i,j]
                                           ,np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) * math.sin(self.phi[i,j])*W[i,j]]


                        self.Rn[i] = self.Rn[i] + math.atan2(self.U_bot[1, i], self.U_bot[0, i]) + math.atan2(self.U_target[1, i], self.U_target[0, i]) * self.move * 0.5 + math.atan2(self.U_obj[1, i], self.U_obj[0, i])

                        # Speeds are calculated based on the energy of neighbors
                        self.s[i] = V[i].sum() + v_obj[i].sum() +  v_target[i] * self.move * 0.5


                        #Speeds are capped.
                        if self.s[i] > 40000:
                            self.s[i] = 40000
 
                        # Bot positions are updated.
                        self.Pn[0, i] = self.Pn[0, i] + self.s[i] * math.cos(self.Rn[i]) * 0.00001
                        self.Pn[1, i] = self.Pn[1, i] + self.s[i] * math.sin(self.Rn[i]) * 0.00001

                        self.O[1, i] = self.O[1,i] + self.obstacle_speeds * (random.random() - 0.5)

                        if self.dynamic_obstacles:
                            
                            if self.O[1,i] > SCREENSIZE[1]:
                                
                                self.obstacle_speeds = - 1.0

                            elif self.O[1,i] < 0:

                                self.obstacle_speeds =  1.0
                        else:

                            self.obstacle_speeds = 0

                        # Bots are drawn

                        pygame.draw.circle(self.screen, black, [int(self.Pn[0, i]), int(self.Pn[1, i])], bot_radius, 1)

                        #Change in energy is calculated
                        self.delta_V = int(0.5*V.sum() - self.V_prev)
                    
                        if self.state != 0:
                            self.move = 1
                            pygame.draw.rect(self.screen,black,(self.target[0] - 25,self.target[1] - 25,50,50), 1)
                        else:
                            self.move = 0

                        
            self.R = self.Rn
            self.P = self.Pn

            pygame.display.update()

            if self.state == 0:
                E_converge.append(0.5*V.sum())
                t_converge.append(time.time() - self.time_0)
            elif self.state == 2:
                E_static.append(0.5*V.sum())
                t_static.append(time.time() - self.time_1)

            elif self.state == 3:
                E_dynamic.append(0.5*V.sum())
                t_dynamic.append(time.time() - self.time_2)
         
            pygame.event.clear()

    def bot_weights(self):

        # The following Configures the Weighted Adjacency Matrix:
        for i in range(N):
            for j in range(N):

                if (i == j):

                    #Diagonal Elements are not important for adjacency because they represent the robot's connections to itself.
                    W[i, j] = 0
                    A[i,j] = 0
                    V[i,j] = 0

                elif (i == j + 1) or (i == j - 1) or ((i == N - 1) and (j == 0)) or ((i == 0) and (j == N - 1)):

                    #Robots should be connected to their immediate neighbors.
                    if connections:
                        pygame.draw.aaline(self.screen, blue, self.P[:, i], self.P[:, j], 1)
                        
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    A[i,j] = 1

                elif (i == j + 2) or (i == j - 2) or ((i == N - 2) and (j == 0)) or ((i == 0) and (j == N - 2)) or ((i == N - 1) and (j == 1)) or ((i == 1) and (j == N - 1)):

                    # Robots have cross diaginal connections to reinforce the polygon formation.
                    phi = (180 - 360 / N)
                    theta = math.radians((180 - phi) / 2)
                    C = 2 * math.cos(theta)
                    A[i,j] = C
                    
                    if connections:
                        pygame.draw.aaline(self.screen, blue, self.P[:, i], self.P[:, j], 1)
                    W[i, j] = K * (1 - C * self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - C * self.min) ** 2

                else:

                        #Non connected bots that get close together considered here.
                        if np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) < ((N*self.a)/(self.b))*self.min:

                            #If the energy is falling, non-connected bots are connected.
                            if (self.delta_V <= 0):
                                if connections:
                                    pygame.draw.aaline(self.screen, red, self.P[:, i], self.P[:, j], 1)
                                W[i, j] = K * (1 -  ((N*self.a)/(self.b))*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                                V[i, j] = 0.5 * K * (np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - ((N*self.a)/(self.b))* self.min) ** 2
                                A[i,j] = -1


                            #if the energy increases the spacing between bots decreases and a counter is increased.
                            else:
                            
                                self.b += 0.01
                                self.counter += 1

                                #If the counter hits a threshold, the counter and spacing divider is reset and the spacing multiplier increaes.
                                if self.counter > 1500:
                                    self.b = 1.0
                                    self.counter = 0
                                    self.a += 1

                                    #Space multiplier is capped
                                    if self.a > 4:
                                        self.a = 1
                                
                                W[i, j] = 0
                                V[i, j] = 0
                                A[i,j] = -1

                                
                        # Non-connected bots that are not close together stay unconnected
                        else:
                            W[i, j] = 0
                            V[i, j] = 0
                            A[i,j] = -1



    def test_cases(self):
        
            for i in range (N):
                self.dist_0[i] = np.linalg.norm(self.target[:] - self.Pn[:, i])
                v_target[i] = np.linalg.norm(self.target[:] - self.Pn[:, i])
                
            self.minimum = int(np.where(self.dist_0 == self.dist_0.min())[0])

            #TEST 1 Convergence to any point:
            print (0.5*V.sum())
            if (0.5*V.sum() < 15000 and self.state == 0):
                
                self.state = 1
                self.target = [SCREENSIZE[0] * 0.75, SCREENSIZE[1]*0.5]

                for i in range(N):
                    self.O[:,i] = [SCREENSIZE[0]/2, 100*i]

                array_converge = [t_converge,E_converge]
                for values in array_converge:
                    writer_converge.writerow(values)

                
            elif (np.linalg.norm(self.target[:] - self.Pn[:, self.minimum]) <= 25 and self.state == 1):

                self.state = 2
                self.target = [SCREENSIZE[0] * 0.05, SCREENSIZE[1]*0.5]

                for i in range(N):
                    self.O[:,i] = [SCREENSIZE[0]*0.4, 100*i - SCREENSIZE[1]*0.2]

                for i in range(4):
                    self.O[:,i] = [SCREENSIZE[0]*0.6, 100*i + SCREENSIZE[1]*0.3]

                self.time_1 = time.time()
  
            elif (np.linalg.norm(self.target[:] - self.Pn[:, self.minimum]) <= 25 and self.state == 2):

                self.state = 3

                for i in range(N):
                    self.O[:,i] = [50*i + SCREENSIZE[0] * 0.25,SCREENSIZE[1]/2 ]
                self.target = [SCREENSIZE[0] * 0.75, SCREENSIZE[1]*0.5]

                self.dynamic_obstacles = True
                self.obstacle_speeds = 1.0

                array_static = [t_static,E_static]
                for values in array_static:
                    writer_static.writerow(values)

                self.time_2 = time.time()

            elif (np.linalg.norm(self.target[:] - self.Pn[:, self.minimum]) <= 25 and self.state == 3):

                self.state = 4
                self.target = [SCREENSIZE[0] * 0.1, SCREENSIZE[1]*0.5]

                self.dynamic_obstacles = True
                self.obstacle_speeds = 1.0

                array_dynamic = [t_dynamic,E_dynamic]
                for values in array_dynamic:
                    writer_dynamic.writerow(values)

            elif (np.linalg.norm(self.target[:] - self.Pn[:, self.minimum]) <= 25 and self.state == 4):
                self.state = 0
                self.dynamic_obstacles = False
                self.O = SCREENSIZE[0] * np.random.rand(Dim, O)
                self.U_obj = np.zeros((Dim, N), dtype=np.int)



    def obj_weights(self):
        if self.state != 0:
            
            #Simulation cycles through all N bots and O obstacles
            for i in range(N):
                for j in range(O):
                    
                    #Obstacle Avoidance Condition:
                    if np.linalg.norm(self.O[:,j] - self.Pn[:,i]) < (self.dynamic[i] + 3*obj_radius):

                        if np.linalg.norm(self.O[:,j] - self.Pn[:,i]) <= (obj_radius):
                            self.num_collisions += 1

                        self.dynamic[i] += 10

                        v_target[i] = 0

                        if self.dynamic[i] > 50:
                            self.dynamic[i] = 50

                        for k in range(N):
                            if A[i,k] != 1:

                                W[i,k] = 0
                                W[k,i] = 0
                                pygame.draw.aaline(self.screen, white, self.P[:, i], self.P[:, k], 2)
                                pygame.draw.aaline(self.screen, white, self.P[:, k], self.P[:, i], 2)

                            elif A[i,k] == 1:

                                if np.linalg.norm(self.O[:,j] - self.Pn[:,i]) <= (obj_radius):

                                    W[i,k] = 0
                                    W[k,i] = 0
                                    pygame.draw.aaline(self.screen, white, self.P[:, i], self.P[:, k], 2)
                                    pygame.draw.aaline(self.screen, white, self.P[:, k], self.P[:, i], 2)

                        w_obj[i,j] = K_obj * self.dynamic[i] * (1 - (self.dynamic[i] + 5*obj_radius) / np.linalg.norm(self.O[:, j] - self.Pn[:, i]))
                        w_obj[j,i] = K_obj * self.dynamic[i] * (1 - (self.dynamic[i] + 5*obj_radius) / np.linalg.norm(self.O[:, j] - self.Pn[:, i]))
                        v_obj[i,j] = 0.5 * self.dynamic[i] * K_obj * (np.linalg.norm(self.O[:, j] - self.Pn[:, i]) - (self.dynamic[i] + 5*obj_radius))**2
                        v_obj[j,i] = 0.5 * self.dynamic[i] * K_obj * (np.linalg.norm(self.O[:, j] - self.Pn[:, i]) - (self.dynamic[i] + 5*obj_radius))**2

                        if connections:
                            pygame.draw.aaline(self.screen, red, self.P[:, i], self.O[:, j], 1)

                    else:
                        self.dynamic[i] -= 5

                        if self.dynamic[i] < 0:
                            self.dynamic[i] = 0

                        w_obj[i,j] = 0
                        w_obj[j,i] = 0
                        v_obj[i,j] = 0
                        v_obj[j,i] = 0

                    # Obstacles are drawn
                    pygame.draw.circle(self.screen, black, [int(self.O[0, j]), int(self.O[1, j])], obj_radius, 1)
        
        
    
if __name__ == '__main__':
    
    try:
        sim = Swarm_Simulation()
        sim.Run()
        
    except KeyboardInterrupt:
        print (" Shutting down simulation...")
        pygame.quit()
        sys.exit()
