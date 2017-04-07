import pygame

from pygame.locals import *

import random

import numpy as np

import math


red = [255, 0, 0]
green = [0, 255, 0]
blue = [0, 0, 255]
white = [255, 255, 255]
black = [0, 0, 0]

SCREENSIZE = [1200, 600]  # Size of our output display

N = 10 #Number of Robots
Dim = 2  #Dimension of search space
bot_radius = 5

running = True

W = np.zeros((N, N), dtype=np.float) # The Swarm Weights Matrix
V = np.zeros((N, N), dtype=np.float) # The Swarm Energy Matrix
A = np.zeros((N, N), dtype=np.float) # The Swarm Adjacency Matrix
K = 500

# The following Configures the Adjacency Matrix:
for i in range(N):
    for j in range(N):

        if (i == j):

            #Diagonal Elements are not important for adjacency because they represent the robot's connections to itself.
            A[i, j] = 0 

        elif (i == j + 1) or (i == j - 1):

            #Robots should be connected to their immediate neighbors.
            A[i,j] = 1 


        elif ((i == N - 1) and (j == 0)) or ((i == 0) and (j == N - 1)):

            #Robot 1 and N are immediate neighbors to each other.
            A[i,j] = 1 

        elif (i == j + 2) or (i == j - 2) or ((i == N - 2) and (j == 0)) or ((i == 0) and (j == N - 2)) or ((i == N - 1) and (j == 1)) or ((i == 1) and (j == N - 1)):

            # Robots have cross diaginal connections to reinforce the polygon formation.
            phi = (180 - 360 / N)
            theta = math.radians((180 - phi) / 2)
            C = 2 * math.cos(theta)
            A[i,j] = C 

        else:

            #All others are treated as special conditions.
            A[i, j] = -1 

class Swarm_Simulation:
    def __init__(self):

        pygame.init()
        self.screen = pygame.display.set_mode(SCREENSIZE)
        self.screen.fill(black)
        pygame.display.set_caption("Rendezvous Simulation")

        self.myfont = pygame.font.SysFont("arial", 25)

        self.P = SCREENSIZE[1] * np.random.rand(Dim, N)  # Initial positions of robots in the search space.
        self.Pn = np.zeros((Dim, N), dtype=np.int) # The updated postions are initialized.

        self.R = np.random.rand(N)  # Initial rotation of robots in 2 pi radians.
        self.Rn = np.zeros((N), dtype=np.int) # The updated rotations are initialized.

        self.phi = np.zeros((N,N), dtype=np.float) #Input angles to robot controllers.
        self.U = np.zeros((Dim, N), dtype=np.int) #  Input desired movement direction for controllers

        self.s = np.zeros((N), dtype=np.float) #Desired speeds of each robot

        self.min = 40 #The target inter-robot distances

        self.V_prev = 0 #The energy of the system.

        self.delta_V = 0 # Change in energy of the system in one program loop.

        self.d = 1.0 #A counter to reduce spacings for non-connected robots

        self.step = 0 #Counter increases whenever the energy begins to increase

        self.m = 1 #Spacer multiplier

    def Run(self):

        while running:
             
            self.screen.fill(black)

            #The intial energy of the system.
            self.V_prev = 0.5*V.sum()

            #The positions and orientations of all bots are updated.
            self.Pn = self.P 
            self.Rn = self.R

            #Simulation cycles through all N bots
            for i in range(N):
                for j in range(N):

                    #Adjacency Matrix sets the Energy and Weight Matrix
                    #Diagonal elements are zero.
                    if (A[i,j] == 0): 
                        W[i, j] = 0
                        V[i, j] = 0

                    #Connections for Bots that are not connected intitally are dynamically set.
                    elif A[i,j]== -1:

                        #Non connected bots that get close together considered here.
                        if np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) < ((N*self.m)/(self.d))*self.min:

                            #If the energy is falling, non-connected bots are connected.
                            if (self.delta_V < 0):
                                
                                pygame.draw.aaline(self.screen, red, self.P[:, i], self.P[:, j], 1)
                                W[i, j] = K * (1 -  ((N*self.m)/(self.d))*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                                V[i, j] = 0.5 * K * (np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - ((N*self.m)/(self.d))* self.min) ** 2

                            #if the energy increases the spacing between bots decreases and a counter is increased.
                            else:
                            
                                self.d += 0.01
                                self.step += 1

                                #If the counter hits a threshold, the counter and spacing divider is reset and the spacing multiplier increaes.
                                if self.step > 1500:
                                    self.d = 1.0
                                    self.step = 0
                                    self.m += 1

                                    #Space multiplier is capped
                                    if self.m > 5:
                                        self.m = 1
                                
                                W[i, j] = 0
                                V[i, j] = 0
                        # Non-connected bots that are not close together stay unconnected
                        else:
                            W[i, j] = 0
                            V[i, j] = 0

                    #Connected bots are assigned weights and energies.
                    else:
                        pygame.draw.aaline(self.screen, blue, self.P[:, i], self.P[:, j], 1)
                        W[i, j] = K * (1 - A[i,j] * self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                        V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - A[i,j]* self.min) ** 2

                    # Robot i's angle input
                    self.phi[i,j] = math.atan2(self.P[1,j] - self.P[1,i], self.P[0,j] - self.P[0,i]) -self.Rn[i] 
                    self.phi[i,i] = 0

                    # Rotation is calculated
                    self.U[0, i] = np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) * math.cos(self.phi[i,j])*W[i,j]
                    self.U[1, i] = np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) * math.sin(self.phi[i,j])*W[i,j]

                    self.Rn[i] = self.Rn[i] + math.atan2(self.U[1, i], self.U[0, i])

                    
                    # Speeds are calculated based on the energy of neighbors
                    self.s[i] = 0.005*V[i].sum()

                    #Speeds are capped.
                    if self.s[i] > 100:
                        self.s[i] = 100
 
                    # Bot positions are updated.
                    self.Pn[0, i] = self.Pn[0, i] + self.s[i] * math.cos(self.Rn[i]) * 0.005
                    self.Pn[1, i] = self.Pn[1, i] + self.s[i] * math.sin(self.Rn[i]) * 0.005

                    # Bots are drawn
                    pygame.draw.circle(self.screen, green, [int(self.Pn[0, i]), int(self.Pn[1, i])], bot_radius, 1)

                    #Change in energy is calculated
                    self.delta_V = int(0.5*V.sum() - self.V_prev)

                    #print 0.5*V.sum()


            self.R = self.Rn
            self.P = self.Pn

            pygame.display.update()
            pygame.event.clear()


if __name__ == '__main__':

    try:
        sim = Swarm_Simulation()
        sim.Run()

    except KeyboardInterrupt:
        print (" Shutting down simulation...")
        pygame.quit()
        sys.exit()
