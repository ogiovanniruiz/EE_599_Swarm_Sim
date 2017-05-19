import pygame

from pygame.locals import *

import numpy as np

import math

import matplotlib.pyplot as plt

import random

import time

red = [255, 0, 0]
green = [0, 255, 0]
blue = [0, 0, 255]
white = [255, 255, 255]
black = [0, 0, 0]

SCREENSIZE = [800, 800]  # Size of our output display

running = True
connections = True

N = 8 #Number of Robots
O = 8 #number of obstacles
bot_radius = 3
obj_radius = 10
Dim = 2  #Dimension of search space

W = np.zeros((N, N), dtype=np.float) # The Swarm Weights Matrix
V = np.zeros((N, N), dtype=np.float) # The Swarm Energy Matrix
A = np.zeros((N, N), dtype=np.float) # The Swarm Adjacency Matrix
w_obj = np.zeros((N,O), dtype=np.float)
v_obj = np.zeros((N,O), dtype=np.float)
v_target = np.zeros((N), dtype=np.float)
K = 500
K_obj = 500

E = []

class Swarm_Simulation:
    def __init__(self):

        pygame.init()
        self.screen = pygame.display.set_mode(SCREENSIZE)
        self.screen.fill(white)
        pygame.display.set_caption("Rendezvous Simulation")

        self.myfont = pygame.font.SysFont("arial", 25)

        self.P = SCREENSIZE[1] * np.random.rand(Dim, N)  # Initial positions of robots in the search space.
        self.Pn = np.zeros((Dim, N), dtype=np.int) # The updated postions are initialized.

        self.min = 50 #The target inter-robot distances.

        self.V_prev = 0 #The energy of the system.

        self.delta_V = 0 # Change in energy of the system in one program loop.

        self.a = np.ones((N), dtype=np.float) #Spacer multiplier.

        self.state = 'A'

        self.start_time = time.time()

    def Run(self):

        while(running):
             
            self.screen.fill(white)

            #The intial energy of the system.
            self.V_prev = 0.5*V.sum()

            #The positions and orientations of all bots are updated.
            self.Pn = self.P

            if self.state == 'A':
                
                sim.A_weights()
                
            elif self.state == 'B':
                
                sim.B_weights()
                
            elif self.state == 'C':
                
                sim.C_weights()


            
            for i in range(N):
                for j in range(N):

                        # Bot positions are updated.
                        self.Pn[:, i] = self.Pn[:, i] + (self.P[:, j] - self.P[:, i]) * 0.00001 * W[i,j]

                        # Bots are drawn
                        pygame.draw.circle(self.screen, black, [int(self.Pn[0, i]), int(self.Pn[1, i])], bot_radius, 1)

                        self.delta_V = int(0.5*V.sum() - self.V_prev)
                        
            self.P = self.Pn

            pygame.display.update()
                   
            pygame.event.clear()

            sim.states()

    def states(self):

        if 0.5*V.sum() < 500 and self.state == 'A':
            self.state = 'B'
            print time.time() - self.start_time
            
        elif 0.5*V.sum() < 500 and self.state == 'B':
            self.state = 'C'
        elif 0.5*V.sum() < 500 and self.state == 'C':
            self.state = 'A'
            print self.state
        
            

    def B_weights(self):

        for i in range(N):
            for j in range(N):
                if (i == j):
                            
                    W[i, j] = 0
                    V[i,j] = 0 
                            
                elif (i == 0) and (j == 1):
                   
                    W[i, j] = K * (1 - 2*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j])) 
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 2*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 0) and (j == 2):

                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 1) and (j == 2):
                    W[i, j] = K * (1 - 1.73*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 1.73*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 1) and ((j == 3) or (j == 4) or (j == 0)):
                    W[i, j] = K * (1 - 2*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 2*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 2) and ((j == 0) or (j == 3)):
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 2) and (j == 1):
                    W[i, j] = K * (1 - 1.73*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 1.73*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 3) and ((j == 1) or (j == 4)):
                   
                    W[i, j] = K * (1 - 2*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 2*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 3) and ((j == 2) or (j == 5)):
                   
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 4) and ((j == 1) or (j == 3) or (j == 6)):
                   
                    W[i, j] = K * (1 - 2*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 2*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 4) and (j == 5):
                    W[i, j] = K * (1 - 1.73*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 1.73*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 5) and (j == 4):
                    W[i, j] = K * (1 - 1.73*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 1.73*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 5) and ((j == 3) or (j == 6)):
                   
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 6) and (j == 5):
                   
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 6) and (j == 4):
                   
                    W[i, j] = K * (1 - 2*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 2*self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                else:

                    
                        if np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) < self.min*self.a[i]:
                            self.a[i] += random.random()
                            if self.a[i] > 4:
                                self.a[i] = 4
                            pygame.draw.aaline(self.screen, red, self.P[:, i], self.P[:, j], 1)
                            W[i, j] = K * (1 -  self.min*self.a[i] / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                            V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min*self.a[i]) ** 2
                            
                        else:
                            W[i, j] = 0
                            V[i,j] = 0
                            self.a[i] -= random.random()*0.1
                            if self.a[i] < 1.0:
                                self.a[i] = 1.0
                            
                                
            
    def C_weights(self):
        for i in range(N):
            for j in range(N):
                if (i == j):
                            
                    W[i, j] = 0
                    V[i,j] = 0 
                            
                elif (i == j + 1) or (i == j - 1):

                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2

                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif ((i == 0) and (j == 7)) or ((i == 1) and (j == 6)) or ((i == 7) and (j == 0)) or ((i == 6) and (j == 1)):

                    W[i, j] = K * (1 - 3*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 3*self.min) ** 2
                    #if connections:
                        #pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif ((i == 0) and (j == 3)) or ((i == 7) and (j == 4)) or ((i == 3) and (j == 0)) or ((i == 4) and (j == 7)):

                    W[i, j] = K * (1 - 2.23*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 2.23*self.min) ** 2
                    #if connections:
                        #pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif ((i == 0) and (j == 5)) or ((i == 7) and (j == 2)) or ((i == 5) and (j == 0)) or ((i == 2) and (j == 7)):

                    W[i, j] = K * (1 - 3.60*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 3.60*self.min) ** 2

                    #if connections:
                        #pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                else:

                    
                        if np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) < self.min*self.a[i]:
                            self.a[i] += random.random()
                            if self.a[i] > 4:
                                self.a[i] = 4
                            pygame.draw.aaline(self.screen, red, self.P[:, i], self.P[:, j], 1)
                            W[i, j] = K * (1 -  self.min*self.a[i] / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                            V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min*self.a[i]) ** 2
                            
                        else:
                            W[i, j] = 0
                            V[i,j] = 0
                            self.a[i] -= random.random()*0.1
                            if self.a[i] < 1.0:
                                self.a[i] = 1.0
                            
                    
                    
        
    def A_weights(self):

        # The following Configures the Weighted Adjacency Matrix:
        for i in range(N):
            for j in range(N):

                if (i == j):
                    
                    W[i, j] = 0
                    V[i,j] = 0 
                    
                elif (i == 0) and ((j == 2) or (j == 1)):
           
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 0) and (j == 4):
                        
                    W[i, j] = K * (1 - 1.73*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 1.73 * self.min) ** 2
                    #if connections:
                        #pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 1) and ((j == 0) or(j == 2) or (j == 3) or (j == 4)):
                        
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 2) and ((j == 0) or(j == 1) or (j == 4) or (j == 5)):

                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 3)and ((j == 1) or(j == 4) or (j == 6)):

                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                        
                elif (i == 4) and ((j == 1) or(j == 2) or (j == 3) or (j == 5)):

                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 4) and (j == 0):

                    W[i, j] = K * (1 - 1.73*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 1.73 * self.min) ** 2
                    #if connections:
                        #pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
                
                elif (i == 5) and ((j == 2) or(j == 4) or (j == 7)):
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)
            

                elif (i == 6) and (j == 3):
                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 6) and (j == 7):
                    W[i, j] = K * (1 - 3*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 3 * self.min) ** 2
                    #if connections:
                        #pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)   

                elif (i == 7) and (j == 5):

                    W[i, j] = K * (1 - self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min) ** 2
                    if connections:
                        pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                elif (i == 7) and (j == 6):
                    W[i, j] = K * (1 - 3*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                    V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - 3 * self.min) ** 2
                    #if connections:
                        #pygame.draw.aaline(self.screen, black, self.P[:, i], self.P[:, j], 1)

                else:
                    
                        if np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) < self.min*self.a[i]:
                            self.a[i] += random.random()
                            if self.a[i] > 4:
                                self.a[i] = 4
                            pygame.draw.aaline(self.screen, red, self.P[:, i], self.P[:, j], 1)
                            W[i, j] = K * (1 -  self.min*self.a[i] / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                            V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - self.min*self.a[i]) ** 2
                            
                        else:
                            W[i, j] = 0
                            V[i,j] = 0
                            self.a[i] -= random.random()*0.1
                            if self.a[i] < 1.0:
                                self.a[i] = 1.0
                            

                            


            
    
if __name__ == '__main__':
    
    try:
        sim = Swarm_Simulation()
        sim.Run()
        
    except KeyboardInterrupt:
        print (" Shutting down simulation...")
        pygame.quit()
        sys.exit()
