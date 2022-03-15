import argparse
from importlib.resources import path
import queue
from typing_extensions import Required
import numpy as np
import cv2
import math
from queue import PriorityQueue
import time
import imageio

# Creating a class Node with visited_node, parent_node and cost as members
class Node:

    def __init__(self, visited_node=False,parent_node=None, cost = math.inf ):
        self.visited_node = visited_node
        self.parent_node = parent_node
        self.cost = cost

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
aStarAlgo('test')
cv2.waitKey(0);cv2.destroyAllWindows()