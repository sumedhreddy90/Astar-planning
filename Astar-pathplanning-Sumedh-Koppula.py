import math
import numpy as np
from heapq import heappush, heappop
import time
import matplotlib.pyplot as plt
import argparse

from sympy import limit

class Map():
    def __init__(self, width = 400, height = 250, radius = 10, clearance = 5, threshold=0.5, theta = 30):
        self.threshold = threshold 
        self.Width = int(width/threshold) 
        self.Height = int(height/threshold) 
        self.radius = radius
        self.clearance = clearance
        self.step = theta 
        self.visited = np.zeros([self.Height, self.Width, 360//theta, 4])
        self.source_x = []
        self.source_y = []
        self.des_x = []
        self.des_y = []

    def mapBuilder(self, plotter):
        # Circle 
        x_circle, y_circle,radii = 300,185, 40 
        x_coordinate = [x_circle+radii*math.cos(i) for i in np.arange(0,2*3.14,0.01)]
        y_coordinate = [y_circle+radii*math.sin(i) for i in np.arange(0,2*3.14,0.01)]
        # Hexagon
        hexagon = [[200,235,235,200,165,165,200],[60,80,120,140,120,80,60]]
        # Boomerang
        boomerang = [[36,115,80,105,36],[190,210,180,100,190]]
        # Plotting obstacles
        plt.plot(boomerang[0],boomerang[1]) 
        plt.plot(hexagon[0],hexagon[1])
        plt.plot(x_coordinate, y_coordinate)
        return plotter
    

    def isObstacle(self, m, n):
        if self.isBoundary(m,n):    
            return False
        elif self.isCircle(m,250-n,(300,185),40):
            return False
        elif self.isHex(m,n):
            return False
        elif self.isBoomerang(m,n):
            return False
        else:
            return True

    def isBoundary(self, m, n):
        if m < (400 - self.radius - self.clearance) and m > (self.radius + self.clearance) and n < (250 - self.radius - self.clearance) and n > (self.radius + self.clearance):
            return False
        return True
    
    def isCircle(self, i, j, center, radius):
        center_x, center_y = center[0], center[1]
        if ((i - center_x) ** 2 + (j - center_y) ** 2) <= (radius + self.radius + self.clearance) ** 2:
            return True
        else:
            return False


    def isHex(self, i, j):
        l1 = (-0.575 * i + 169+10 - j) <= 0
        l2 = (160-10 - i) <= 0
        l3 = (0.575 * i + 31-10 - j) >= 0
        l4 = (-0.575 * i + 261+10 - j) >= 0
        l5 = (240-10 - i) >= 0
        l6 = (0.575 * i - 61+10 - j) <= 0

        if l1 and l2 and l3 and l4 and l5 and l6:
            return True
        else:
            return False

    def isBoomerang(self, i, j):
        l1 = (0.316 * i + 178.608 - j) >= 0
        l2 = (0.857 * i + 106.429 - 10 - j) <= 0
        lmid = (-0.114 * i + 189.091 - j) <= 0
        l3 = (-3.2 * i + 450 - j) >= 0
        l4 = (-1.232 * i + 220.348 - j) <= 0

        if (l1 and l2 and lmid) or (l3 and l4 and not lmid):
            return True
        else:
            return False

    def isExplored(self, n):
        x = int(round(n[1]/self.threshold))
        y = int(round(n[2]/self.threshold))
        a = int(n[3]//self.step)
        if self.visited[y, x, a,3] != 0:
            # True if the node is visited
            return True 

        else:
            # False if the node is not visited
            return False

    def getExploredList(self, n):
        x = int(round(n[1]/self.threshold))
        y = int(round(n[2]/self.threshold))
        a = int(n[3]//self.step)
        return self.visited[y, x, a, :]

    def exploredList(self, n, p_node):
        x = int(round(n[1]/self.threshold))
        y = int(round(n[2]/self.threshold))
        a = int(n[3]//self.step)
        self.source_x.append(p_node[1])
        self.source_y.append(p_node[2])
        self.des_x.append(n[1] - p_node[1]) 
        self.des_y.append(n[2] - p_node[2])
        self.visited[y, x, a, :] = np.array(p_node)
        return

    def mapPlotter(self, path):
        plt.ion()
        fig, ax = plt.subplots()
        ax = self.mapBuilder(ax)
        for i in range(1,len(self.source_x)+1,3000):
            plt.xlim(0,400)
            plt.ylim(0,250)
            q = ax.quiver(self.source_x[i:(i+3000)], self.source_y[i:(i+3000)], 
                self.des_x[i:(i+3000)], self.des_y[i:(i+3000)], units='xy' ,
                scale = 1, headwidth = 0.1, headlength=0,
                width=0.2)
            plt.pause(0.0001)

        X,Y,U,V = [],[],[],[] 
        for i in range(len(path)-1):
            X.append(path[i][1])
            Y.append(path[i][2])
            U.append(path[i+1][1] - path[i][1])
            V.append(path[i+1][2] - path[i][2])
            
        for i in range(len(X)):
            plt.xlim(0,400)
            plt.ylim(0,250)
            q = ax.quiver(X[i], Y[i], U[i], V[i], units='xy', 
                scale=1, color='y', headwidth = 0.1, 
                headlength=0, width = 0.7)
            plt.pause(0.0001)
        plt.ioff()
        plt.show()

class AStarAlgorithm():
    def __init__(self, source, destination, theta = 30, stepSize = 1, goalThreshold = 1.5,
        width = 400, height = 250, threshold = 0.5, radius = 10, clearance = 5):
        self.source = source
        self.destination = destination
        ##### n = [ x , y , a_i , cost ]
        self.nodeData = []
        self.node_data = []
        self.theta = theta
        self.stepSize = stepSize
        self.goalThreshold = goalThreshold
        self.actionSpace()
        self.map_with_obstacles = Map(width, height, radius = radius, clearance = clearance, threshold=threshold, theta=self.theta)


    def actionSpace(self):
        self.action_list = []
        for a_i in np.arange(0, 360, self.theta):
            angle = math.radians(a_i)
            x = self.stepSize*math.cos(angle)
            y = self.stepSize*math.sin(angle)
            cost_to_come = 1
            #### a  --- [ x , y , a_i , cost ] ----
            self.action_list.append([x, y, a_i, cost_to_come])
        pass

    def isValid(self):
    #writing the condition for the case when start or destination n are defined in an map_with_obstacles
        if not self.map_with_obstacles.isObstacle(self.destination[0], 250-self.destination[1]):
            print("destination within obstacle field")
            return False
        elif not self.map_with_obstacles.isObstacle(self.source[0], 250-self.destination[1]):
            print("source position within obstacle field")
            return False
        else:
            c = math.sqrt((self.source[0] - self.destination[0])**2 + (self.source[1] - self.destination[1])**2)
            heappush(self.node_data, [c, self.source[0], self.source[1], self.source[2], 0])
            self.nodeData.append([self.source[0], self.source[1], self.source[2], 0])
            return True
 #defining heuristic function as euclidian distance between c_node n and destination
    def heuristic_function(self, c_node):
        heuristic_ecludian = math.sqrt((c_node[1] - self.destination[0])**2 + (c_node[2] - self.destination[1])**2)
        return heuristic_ecludian
# function to check if the visited point is inside threshold area around the destination or not
    def goalState(self, c_node):  
        x, y = c_node[1], c_node[2]
        if (x - destination[0])**2 + (y - destination[1])**2 <= (self.goalThreshold)**2:
            return True
        else:
            return False

    def backTracePath(self, present_node):
        trace_path = []
        trace_node = present_node[:4]
        trace_path.append(trace_node)
        while trace_node[1:] != self.source:
            trace_node = list(self.map_with_obstacles.getExploredList(trace_node))
            trace_path.append(trace_node)
        trace_path.reverse()
        return trace_path

    def findGoal(self):
        if self.isValid():
            while len(self.node_data)>0:
                present_node = heappop(self.node_data)
                previous_cost, previous_cost_to_come = present_node[0], present_node[4]
                if self.goalState(present_node):
                    print(" Robot reached destination location ")
                    print("Goal reached at : ",present_node)
                    trace_path = self.backTracePath(present_node)
                    print(trace_path)
                    self.map_with_obstacles.mapPlotter(trace_path)
                    return
                for a in self.action_list:
                    ##### n = [ x , y , a_i , cost]
                    ##### node_data = [ cost , selfID , parentID ]
                    X = present_node[1] + a[0]
                    Y = present_node[2] + a[1]
                    A = a[2]
                    updated_node = [0, X, Y, A, 0]
                    updated_cost_to_come = previous_cost_to_come + a[3]
                    updated_node[4] = updated_cost_to_come
                    cost_to_go = self.heuristic_function(updated_node)
                    if self.map_with_obstacles.isObstacle(X, Y):
                        if not self.map_with_obstacles.isExplored(updated_node):
                            # Adding unvisted to node data array
                            present_node[0] = updated_cost_to_come
                            self.map_with_obstacles.exploredList(updated_node, present_node[:4])
                            updated_node[0] = updated_cost_to_come + cost_to_go
                            heappush(self.node_data, updated_node)
                        else: # checking cost for visited nodes
                            last_visited_node = self.map_with_obstacles.getExploredList(updated_node)
                            previous_cost = last_visited_node[0]
                            if previous_cost > updated_cost_to_come:
                                present_node[0] = updated_cost_to_come
                                self.map_with_obstacles.exploredList(updated_node, present_node[:4])
           
                         

        print("Error reaching goal postion")
        return

# Input source, destination location and other parameters from the args/ commandline
input_parser = argparse.ArgumentParser()
input_parser.add_argument('--Start', default="[50,30,60]", help='Please enter a valid start location')
input_parser.add_argument('--End', default="[380,200,0]", help='Please enter a valid destination location')
input_parser.add_argument('--RobotRadius', default=10, help='')
input_parser.add_argument('--ObjectClearance', default=5, help='Give robot clearance with objects')
input_parser.add_argument('--IsAnimation', default=1, help='1 - True 0 - false')
input_parser.add_argument('--FramesPerSecond', default=30, help='frame rate')
input_parser.add_argument('--theta', default=30, help='action for a_i')
input_parser.add_argument('--Step', default=2, help='Step size')
input_parser.add_argument('--Threshold', default=0.5, help='Threshold value for approximation')
input_parser.add_argument('--GoalThreshold', default=1.5, help='Circle radius for the goal point')
args = input_parser.parse_args()
args = input_parser.parse_args()

start_location = args.Start
end_location = args.End
radius = int(args.RobotRadius)
clearance = int(args.ObjectClearance)
animation = int(args.IsAnimation)
framerate = int(args.FramesPerSecond)
theta = int(args.theta)
StepSize = int(args.Step)
Threshold = float(args.Threshold)
GoalThreshold = float(args.GoalThreshold)

source = [int(i) for i in start_location[1:-1].split(',')]
destination = [int(i) for i in end_location[1:-1].split(',')] 

path_planner = AStarAlgorithm(source, destination, theta=theta, stepSize=StepSize,
    goalThreshold = GoalThreshold, width = 400, height = 250, threshold = Threshold,
    radius=radius, clearance=clearance)
path_planner.findGoal()
