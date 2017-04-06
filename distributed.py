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

N = 12#Number of Robots
Dim = 2  #Dimension of search space
bot_radius = 5

running = True

W = np.zeros((N, N), dtype=np.float) # The Swarm Position Weights Matrix
V = np.zeros((N, N), dtype=np.float) # The Swarm Position Weights Matrix
A = np.zeros((N, N), dtype=np.float) # The Swarm Position Weights Matrix
K = 500

for i in range(N):
    for j in range(N):

        if (i == j):

            A[i, j] = 0

        elif (i == j + 1) or (i == j - 1):

            A[i,j] = 1


        elif ((i == N - 1) and (j == 0)) or ((i == 0) and (j == N - 1)):

            A[i,j] = 1

        elif (i == j + 2) or (i == j - 2) or ((i == N - 2) and (j == 0)) or ((i == 0) and (j == N - 2)) or ((i == N - 1) and (j == 1)) or ((i == 1) and (j == N - 1)):

            phi = (180 - 360 / N)
            theta = math.radians((180 - phi) / 2)
            C = 2 * math.cos(theta)
            A[i,j] = C

        else:

            A[i, j] = -1

class Swarm_Simulation:
    def __init__(self):

        pygame.init()
        self.screen = pygame.display.set_mode(SCREENSIZE)
        self.screen.fill(black)
        pygame.display.set_caption("Rendezvous Simulation")

        self.myfont = pygame.font.SysFont("arial", 25)

        self.P = SCREENSIZE[1] * np.random.rand(Dim, N)  # Initial positions of robots in a 1000x1000 unit space
        self.Pn = np.zeros((Dim, N), dtype=np.int)

        self.R = np.random.rand(N)  # Initial rotation of robots in 360 degrees
        self.Rn = np.zeros((N), dtype=np.int)

        self.phi = np.zeros((N,N), dtype=np.float)
        self.U = np.zeros((Dim, N), dtype=np.int)

        self.s = np.zeros((N), dtype=np.float)

        self.min = 60

        self.V_prev = 0

        self.delta_V = 0

        self.step = 1.0

    def Run(self):

        while running:


            self.V_prev = 0.5*V.sum()
            self.screen.fill(black)

            self.Pn = self.P
            self.Rn = self.R

            for i in range(N):
                for j in range(N):

                    if (A[i,j] == 0):
                        W[i, j] = 0
                        V[i, j] = 0
                        
                    elif A[i,j]== -1:
                    
                        if np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) < (N/(self.step))*self.min:

                            if (self.delta_V <= 0):
                                
                                pygame.draw.aaline(self.screen, red, self.P[:, i], self.P[:, j], 1)
                                W[i, j] = K * (1 -  (N/(self.step))*self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                                V[i, j] = 0.5 * K * (np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - (N/3.0)* self.min) ** 2
                            else:
                            
                                self.step += 0.01
                                W[i, j] = 0
                                V[i, j] = 0
                        else:
                            W[i, j] = 0
                            V[i, j] = 0

                    else:
                        pygame.draw.aaline(self.screen, blue, self.P[:, i], self.P[:, j], 1)
                        W[i, j] = K * (1 - A[i,j] * self.min / np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]))
                        V[i, j] = 0.5 * K *(np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) - A[i,j]* self.min) ** 2

                    self.phi[i,j] = math.atan2(self.P[1,j] - self.P[1,i], self.P[0,j] - self.P[0,i]) -self.Rn[i] # Robot i's angle input
                    self.phi[i,i] = 0

                    self.U[0, i] = np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) * math.cos(self.phi[i,j])*W[i,j]
                    self.U[1, i] = np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) * math.sin(self.phi[i,j])*W[i,j]

                    self.Rn[i] = self.Rn[i] + math.atan2(self.U[1, i], self.U[0, i])

                    self.s[i] = 0.0005*V[i].sum()

                    if self.s[i] > 100:
                        self.s[i] = 100

                    self.Pn[0, i] = self.Pn[0, i] + self.s[i] * math.cos(self.Rn[i]) * 0.005
                    self.Pn[1, i] = self.Pn[1, i] + self.s[i] * math.sin(self.Rn[i]) * 0.005 

                    pygame.draw.circle(self.screen, green, [int(self.Pn[0, i]), int(self.Pn[1, i])], bot_radius, 1)
                    
                    self.delta_V = int(0.5*V.sum() - self.V_prev)


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
