import pygame

from pygame.locals import *

import numpy as np

import math

red = [255, 0, 0]
green = [0, 255, 0]
blue = [0, 0, 255]
white = [255, 255, 255]
black = [0, 0, 0]

SCREENSIZE = [1200, 600]  # Size of our output display

running = True

N = 8 #Number of Robots
O = 8 #number of obstacles
bot_radius = 3
obj_radius = 15
Dim = 2  #Dimension of search space

W = np.zeros((N, N), dtype=np.float) # The Swarm Weights Matrix
V = np.zeros((N, N), dtype=np.float) # The Swarm Energy Matrix
A = np.zeros((N, N), dtype=np.float) # The Swarm Adjacency Matrix
w_obj = np.zeros((N,N), dtype=np.int)
#v_obj = np.zeros((N,N), dtupe=np.int)
K = 500

class Swarm_Simulation:
    def __init__(self):

        pygame.init()
        self.screen = pygame.display.set_mode(SCREENSIZE)
        self.screen.fill(black)
        pygame.display.set_caption("Rendezvous Simulation")

        self.myfont = pygame.font.SysFont("arial", 25)

        self.O = 600 * np.random.rand(Dim, O)# Obstacle Positions

        self.P = SCREENSIZE[1] * np.random.rand(Dim, N)  # Initial positions of robots in the search space.
        self.Pn = np.zeros((Dim, N), dtype=np.int) # The updated postions are initialized.

        self.R = np.random.rand(N)  # Initial rotation of robots in 2 pi radians.
        self.Rn = np.zeros((N), dtype=np.int) # The updated rotations are initialized.

        self.R_obj = np.random.rand(N)  # Initial rotation of robots in 2 pi radians.
        self.Rn_obj = np.zeros((N), dtype=np.int) # The updated rotations are initialized.
        
        self.phi = np.zeros((N,N), dtype=np.float) #Input angles to robot controllers.
        self.phi_obj = np.zeros((N,N), dtype=np.float) #Input angles to robot controllers.
        self.U = np.zeros((Dim, N), dtype=np.int) #  Input desired movement direction for controllers.
        self.U_obj = np.zeros((Dim, N), dtype=np.int) #  Input desired movement direction for controllers.

        self.s = np.zeros((N), dtype=np.float) #Desired speeds of each robot.

        self.min = 40 #The target inter-robot distances.

        self.base = 40

        self.V_prev = 0 #The energy of the system.

        self.delta_V = 0 # Change in energy of the system in one program loop.

        self.d = 1.0 #A counter to reduce spacings for non-connected robots.

        self.m = 1 #Spacer multiplier.

        self.rise = 0 #Counter increases whenever the energy begins to increases.

        self.dist = np.zeros((N), dtype=np.float)

        self.minimum = 0

        self.leader = np.zeros((N), dtype=np.int)

        self.target = [500,500]

        self.state = 0

    def Run(self):

        while(running):
             
            self.screen.fill(black)

            #The intial energy of the system.
            self.V_prev = 0.5*V.sum()

            #The positions and orientations of all bots are updated.
            self.Pn = self.P 
            self.Rn = self.R
            self.Rn_obj = self.R_obj

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
                                self.rise += 1

                                #If the counter hits a threshold, the counter and spacing divider is reset and the spacing multiplier increaes.
                                if self.rise > 1500:
                                    self.d = 1.0
                                    self.rise = 0
                                    self.m += 1

                                    #Space multiplier is capped
                                    if self.m > 4:
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

                    #print self.rise
                    #===================================================================================================
                    #Obstacle Code:
                    if np.linalg.norm(-self.Pn[:,i] + self.O[:,j]) <= (6*obj_radius):

                        self.leader[i] = 0
                        #self.min += 0.01
                        #v_obj[i,j] = 0.5 * K * (np.linalg.norm(-self.Pn[:, i] + self.O[:, j]) - (obj_radius*5)) ** 2
                        #print v_obj
                        w_obj[i,j] = K * 10000*(1 - (9*obj_radius) / np.linalg.norm(-self.Pn[:, i] + self.O[:, j]))
                        pygame.draw.aaline(self.screen, blue, self.P[:, i], self.O[:, j], 1)

                    else:
                        self.leader[i]  = 1
                        w_obj[i,j] = 0
                        #v_obj[i,j] = 0

                    w_obj[i,i] = 0
                    #v_obj[i,i] = 0

                    # Robot and obstacle rotation input
                    self.phi_obj[i,j] = math.atan2(-self.P[1,i] + self.O[1,j], -self.P[0,i] + self.O[0,j]) - self.Rn[i] 
                    self.phi_obj[i,i] = 0

                    #=========================================================================================================
                    
                    # Inter Robot i's angle input
                    self.phi[i,j] = math.atan2(self.P[1,j] - self.P[1,i], self.P[0,j] - self.P[0,i]) - self.Rn[i] 
                    self.phi[i,i] = 0
                
                    # Inter robot Rotation is calculated
                    self.U[0, i] = np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) * math.cos(self.phi[i,j])*W[i,j] + np.linalg.norm(-self.Pn[:, i] + self.O[:, j]) * math.cos(self.phi_obj[i,j])*w_obj[i,j]
                    self.U[1, i] = np.linalg.norm(self.Pn[:, i] - self.Pn[:, j]) * math.sin(self.phi[i,j])*W[i,j] + np.linalg.norm(-self.Pn[:, i] + self.O[:, j]) * math.sin(self.phi_obj[i,j])*w_obj[i,j]

                    self.Rn[i] = self.Rn[i] + math.atan2(self.U[1, i], self.U[0, i]) 

                    # Speeds are calculated based on the energy of neighbors
                    self.s[i] = 0.005 *V[i].sum()

                    #Speeds are capped.
                    if self.s[i] > 100:
                        self.s[i] = 100
 
                    # Bot positions are updated.
                    self.Pn[0, i] = self.Pn[0, i] + self.s[i] * math.cos(self.Rn[i]) * 0.005  + self.leader[i] * (self.target[0] - self.P[0,i]) * 0.0002
                    self.Pn[1, i] = self.Pn[1, i] + self.s[i] * math.sin(self.Rn[i]) * 0.005  + self.leader[i] * (self.target[1] - self.P[1,i]) * 0.0002

                    # Bots are drawn
                    pygame.draw.circle(self.screen, green, [int(self.Pn[0, i]), int(self.Pn[1, i])], bot_radius, 1)

                    # Obstacles are drawn
                    pygame.draw.circle(self.screen, white, [int(self.O[0, i]), int(self.O[1, i])], obj_radius, 1)

                    #Change in energy is calculated
                    self.delta_V = int(0.5*V.sum() - self.V_prev)
                    
                    if self.state != 0:

                        pygame.draw.aaline(self.screen, red, self.target[:], self.P[:,self.minimum], 1)
                        pygame.draw.rect(self.screen,white,(self.target[0] - 25,self.target[1] - 25,50,50), 1)

            for i in range (N):
                self.dist[i] = np.linalg.norm(self.target[:] - self.Pn[:, i])
                    
            self.minimum = int(np.where(self.dist == self.dist.min())[0])  

            self.R_obj = self.Rn_obj
            self.R = self.Rn
            self.P = self.Pn

            if (0.5*V.sum() < 5000 and self.state == 0):
                self.state = 1
                self.target = [800,500]
 
            elif (np.linalg.norm(self.target[:] - self.Pn[:, self.minimum]) <= 25 and self.state == 1):

                self.state = 2
                self.min = self.base
                self.target = [800,100]

                for i in range (N):
                    self.dist[i] = np.linalg.norm(self.target[:] - self.Pn[:, i])

                self.minimum = int(np.where(self.dist == self.dist.min())[0])

            elif (np.linalg.norm(self.target[:] - self.Pn[:, self.minimum]) <= 25 and self.state == 2):

                self.state = 3
                self.min = self.base
                self.target = [100,100]

                for i in range (N):
                    self.dist[i] = np.linalg.norm(self.target[:] - self.Pn[:, i])

                self.minimum = int(np.where(self.dist == self.dist.min())[0])

            elif (np.linalg.norm(self.target[:] - self.Pn[:, self.minimum]) <= 25 and self.state == 3):

                self.state = 4
                self.min = self.base
                self.target = [100,500]

                for i in range (N):
                    self.dist[i] = np.linalg.norm(self.target[:] - self.Pn[:, i])

                self.minimum = int(np.where(self.dist == self.dist.min())[0])

            elif (np.linalg.norm(self.target[:] - self.Pn[:, self.minimum]) <= 25 and self.state == 4):
                self.state = 0
                self.min = self.base


            pygame.display.update()
                   
            pygame.event.clear()

    def polygon(self):

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
    
if __name__ == '__main__':
    
    try:
        sim = Swarm_Simulation()
        sim.polygon()
        sim.Run()
        
    except KeyboardInterrupt:
        print (" Shutting down simulation...")
        pygame.quit()
        sys.exit()
