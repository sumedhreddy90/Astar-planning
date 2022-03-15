import argparse
from importlib.resources import path
import queue
from typing_extensions import Required
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

    def findGoal(self):
        if self.isValid():
           
            return True    

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
goal = [int(i) for i in end_location[1:-1].split(',')]

planner = aStarAlgo('test')
planner.findPlan()
cv2.waitKey(0);cv2.destroyAllWindows()