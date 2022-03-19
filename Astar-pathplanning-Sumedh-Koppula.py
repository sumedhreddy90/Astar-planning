import argparse
from importlib.resources import path
import queue
from typing_extensions import Required
from xml.sax.saxutils import prepare_input_source
from keyring import set_keyring
import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop
import math
from queue import PriorityQueue
import time
import imageio
from sympy import true

class Map:
    def __init__(self, width = 400, height = 250, radius = 1, clearance = 1, threshold=0.5, theta = 30):
        self.threshold = threshold
        self.map_width = int(width/threshold) 
        self.map_height = int(height/threshold)
        self.radius = radius
        self.clearance = clearance
        self.theta = theta
        self.visited = np.zeros([self.map_height, self.map_width, 360//theta, 4])
        self.source_x = []
        self.source_y = []
        self.destination_x = []
        self.destination_y = []
    
    def mapBuilder(self, plotter):
        # Circle 
        x_circle, y_circle,radii = 300,65, 40 
        x_coordinate = [x_circle+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        y_coordinate = [y_circle+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        # Hexagon
        hexagon = [[200,235,235,200,165,165,200],[190,170,130,110,130,170,190]]
        # Boomerang
        boomerang = [[36,115,80,105,36],[65,40,70,150,65]]
        # Plotting obstacles
        plt.plot(boomerang[0],boomerang[1]) 
        plt.plot(hexagon[0],hexagon[1])
        plt.plot(x_coordinate, y_coordinate)
        return plotter

    def isObstacle(self, m, n):

        # Check for obstacles
        return
    
    def isExplored(self, node):
        x = int(round(node[1]/self.threshold))
        y = int(round(node[2]/self.threshold))
        a = int(node[3]//self.theta)
        if self.visited[y, x, a, 3] !=0:
            return True
        else:
            return False
    def mapPlotter(self, trace_path):
        plt.ion()
        _, axis = plt.subplot()
        axis = self.mapBuilder(axis)
        for m in range(1, len(self.source_x)+1, 3000):
            plt.xlim(0,300)
            plt.xlim(0,200)
            plot = axis.quiver(self.source_x[m:(m+3000)], self.source_y[m:(m+3000)], 
                self.destination_y[m:(m+3000)], self.destination_y[m:(m+3000)], units='xy' ,
                scale = 1, headwidth = 0.1, headlength=0,
                width=0.2)
            plt.pause(0.0001)
        x_source,y_source,x_destination,y_destination = [],[],[],[] 
        for i in range(len(path)-1):
            x_source.append(path[i][1])
            y_source.append(path[i][2])
            x_destination.append(path[i+1][1] - path[i][1])
            y_destination.append(path[i+1][2] - path[i][2])
            
        for i in range(len(x_source)):
            # plt.cla()
            plt.xlim(0,300)
            plt.ylim(0,200)
            q = axis.quiver(x_source[i], y_source[i], x_destination[i], y_destination[i], units='xy', 
                scale=1, color='r', headwidth = 0.1, 
                headlength=0, width = 0.7)
            plt.pause(0.0001)
        plt.ioff()
        plt.show()
    def exploredList(self, n, parent):
        x = int(round(n[1]/self.threshold))
        y = int(round(n[2]/self.threshold))
        a = int(n[3]//self.theta)
        self.source_x.append(parent[1])
        self.source_y.append(parent[2])
        self.destination_x.append(n[1] - parent[1]) 
        self.destination_y.append(n[2] - parent[2])
        self.explored[y, x, a, :] = np.array(parent)
        return

    def getExploredList(self, node):
        x = int(round(node[1]/self.threshold))
        y = int(round(node[2]/self.threshold))
        a = int(node[3]//self.theta)
        return self.explored[x, y, a, :]

class AStarAlgorithm:

    def __init__(self, source, destination, theta= 30, step= 1, threshold = 0.5, width=400, height=250, goal_threshold= 1.5, radius=10, clearance=5):
        self.source = source
        self.destination = destination
        self.node_data = []
        self.theta = theta
        self.step_function = step
        self.threshold = threshold
        self.goal_threshold = goal_threshold
        self.actionSpace()
        self.obstacle_map = Map(width, height, radius= radius, clearance =clearance, threshold=threshold, thetaStep=self.theta)


    def actionSpace(self):
        self.actionSpace = []
        for i in np.arange(0, 360, self.theta):
            i = math.radians(i)
            x = self.step_function*math.cos(i)
            y = self.step_function*math.sin(i)
            cost_to_come = 1
            self.actionSpace.append([x, y, i, cost_to_come])
    
    def isValid(self):

        return True
    def goalState(self, present_state):
        x, y = present_state[1], present_state[2]
        if (x- destination[0])**2 + (y -destination[1])**2 <= (self.goal_threshold)**2:
            return True
        else:
            return False

    #ecludian distance
    def heuristic_function(self, present):
        distance = math.sqrt((present[1] - self.destination[0])**2 + (present[2] - self.destination[1])**2)
        return distance
    def findGoal(self):
        if self.isValid():
           while len(self.node_data)>0:
               present_node = heappop(self.node_data)
               previous_cost, previous_cost_to_come = present_node[0], present_node[4]
               if self.goalState(present_node):
                   self.goal_status = True
                   print("The Robot has reached the Destination")
                   print(present_node)
                   # Backtracing the path from goal location to source location
                   trace = self.backTracePath(present_node)
                   self.obstacle_map.mapPlotter(trace)
                   return
               for a in self.actionSpace:
                    present_node_x = present_node[1] + a[0]
                    present_node_y = present_node[2] + a[1]
                    init_node = a[2]
                    present_node_data = [0, present_node_x, present_node_y, init_node, 0]
                    present_cost_to_come = previous_cost_to_come + a[3]
                    init_node[4] = present_cost_to_come
                    cost_to_go = self.heuristic_function(init_node)
                    if self.obstacle_map.isObstacle(present_node_x, present_node_y):
                        if not self.obstacle_map.isExplored(init_node):
                            present_node[0] = present_cost_to_come
                            self.obstacle_map.exploredList(init_node,present_node[:4])
                            init_node[0] = present_cost_to_come + cost_to_go
                            heappush(self.node_data, init_node)
                        else:
                            past_explored_node = self.obstacle_map.getExploredList(init_node)
                            previous_cost =  past_explored_node[0]
                            if previous_cost > present_cost_to_come:
                                present_node[0] = present_cost_to_come
                                self.obstacle_map.exploredList(init_node, present_node[:4])

        print("Error reaching goal postion")
        return 

def aStarAlgo(input):
     
    return

# Input source, destination location and other parameters from the args/ commandline
input_parser = argparse.ArgumentParser()
input_parser.add_argument('--Start', default="[0,0,60]", help='Please enter a valid start location')
input_parser.add_argument('--End', default="[150,180,0]", help='Please enter a valid destination location')
input_parser.add_argument('--RobotRadius', default=1, help='')
input_parser.add_argument('--ObjectClearance', default=1, help='Give robot clearance with objects')
input_parser.add_argument('--IsAnimation', default=1, help='1 - True 0 - false')
input_parser.add_argument('--FramesPerSecond', default=30, help='frame rate')
input_parser.add_argument('--theta', default=30, help='action for angle')
input_parser.add_argument('--Step', default=2, help='Step size')
input_parser.add_argument('--Threshold', default=0.5, help='Threshold value for approximation')
input_parser.add_argument('--GoalThreshold', default=2, help='Circle radius for the goal point')
Argument = input_parser.parse_args()

Argument = input_parser.parse_args()

start_location = Argument.Start
end_location = Argument.End
radius = int(Argument.RobotRadius)
clearance = int(Argument.ObjectClearance)
Isanimation = int(Argument.IsAnimation)
ff = int(Argument.FramesPerSecond)
theta = int(Argument.theta)
step = int(Argument.Step)
threshold = float(Argument.Threshold)
goal_threshold = float(Argument.GoalThreshold)

source = [int(i) for i in start_location[1:-1].split(',')]
destination = [int(i) for i in end_location[1:-1].split(',')]

planner = aStarAlgo('test')