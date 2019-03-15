import numpy as np
import pybullet as p
import itertools
import os, inspect
import pybullet_data
from robot import Robot
    
class World():
    def __init__(self):
        # create the physics simulator
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0,0,-9.81)
        
        self.max_communication_distance = 10.0

        # We will integrate every 4ms (250Hz update)
        self.dt = 1./250.
        p.setPhysicsEngineParameter(self.dt, numSubSteps=1)

        #FOR STADIUM PLANE
        self.planeId = p.loadSDF(os.path.join(pybullet_data.getDataPath(),"stadium.sdf"))
        print('plane id',self.planeId)
        p.changeDynamics(self.planeId[0], -1, lateralFriction=2, rollingFriction=0)

        # FOR SIMPLE PLANE
        # self.planeId = p.loadURDF("../models/plane.urdf")
        # print('plane id',self.planeId)
        # p.changeDynamics(self.planeId, -1, lateralFriction=5., rollingFriction=0)

        #FOR ADDING GOAL
        # self.goalId = p.loadURDF("../models/goal.urdf")
        
        p.resetDebugVisualizerCamera(7.0,50.0, -35.0, (1., 1., 0.0))
        
        # create 6 robots
        self.robots = []
        for (i,j) in itertools.product(range(2), range(2)):
            self.robots.append(Robot([1. * i + 0.5, 1. * j - 0.5, 0.3], 2*i+j, self.dt))
            p.stepSimulation()
        
        self.time = 0.0
        
        self.stepSimulation()
        self.stepSimulation()

    def reset(self):
        """
        Resets the position of all the robots
        """
        for r in self.robots:
            r.reset()
        p.stepSimulation()
        
    def stepSimulation(self):
        """
        Simulates one step simulation
        """
        
        # for each robot construct list of neighbors
        for r in self.robots:
            r.neighbors = [] #reset neighbors
            r.messages_received = [] #reset message received
            pos1, or1 = r.get_pos_and_orientation()
            for j,r2 in enumerate(self.robots):
                if(r.id != r2.id):
                    pos2, or2 = r2.get_pos_and_orientation()
                    if(np.linalg.norm(pos1-pos2) < self.max_communication_distance):
                        r.neighbors.append(j)
        
        # for each robot send and receive messages
        for i,r in enumerate(self.robots):
            for msg in r.messages_to_send:
                if msg[0] in r.neighbors: #then we can send the message
                    self.robots[msg[0]].messages_received.append([i, msg[1]]) #add the sender id
            r.messages_to_send = []
        
        # update the controllers
        if self.time > 1.0:
            for r in self.robots:
                r.compute_controller()
        
        # do one simulation step
        p.stepSimulation()
        self.time += self.dt
        
