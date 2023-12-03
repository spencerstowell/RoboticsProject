# Description: This file defines a class for Rapidly Exploring Random Trees (RRTs)
# specifically for the purpose of path planning for a robot arm.

# Import needed libraries
import numpy as np
import math
import random
import matplotlib.pyplot as plt

# Define the class for the RRT algorithm
class RRT:
    # Define the initialization function
    # The 
    def __init__(self, start, goal, obstacles, stepsize, max_iter, arm):
        """
        Accepts arguments start, goal, obstacles, stepsize, and max_iter
        start - the starting point of the end effector in 3D cartesian coordinates
        goal - the goal point of the end effector in 3D cartesian coordinates
        obstacles - a list of obstacles, where each obstacle is defined in 3D cartesian coordinates with a radius
            ex - [x, y, z, radius]
            Future idea - include another list parameter specifying spherical vs boxy obstacles
        stepsize - the stepsize for the RRT algorithm
        max_iter - the maximum number of iterations for the RRT algorithm
        arm - a robot arm object with all parameters for calculating intermediate points
        """
        self.arm = arm

        # Initialize the start and goal points
        self.start = start
        self.goal = goal

        # Initialize the obstacles
        self.obstacles = obstacles

        # Initialize the stepsize and max iterations
        self.stepsize = stepsize
        self.max_iter = max_iter

        # Initialize the tree
        self.tree = [start]

        # Initialize the path
        self.path = []

        # Initialize the path_found flag
        self.path_found = False

    # Define the function to check if a point is in an obstacle
    def in_obstacle(self, point):
        # Initialize the in_obstacle flag
        in_obstacle = False

        # Iterate through the obstacles
        for i in self.obstacles:
            obs = self.obstacles[i]
            # Check if the point is in the obstacle by comparing the magnitude of:
            # the distance of the end effector to the center of the obstacle
            # to:
            # the radius of the obstacle
            # This is inherently designed to work with spherical obstacles
            if (np.sqrt((point[0] - obs[i,0])^2 + (point[1] - obs[i,1])^2 + (point[2] - obs[i,2])^2) < obs[i,3]):
                # Set the in_obstacle flag
                in_obstacle = True

        # Return the in_obstacle flag
        return in_obstacle
    
    def in_workspace(self, point):
        # Initialize the in_workspace flag
        in_workspace = True
        # Check if the point is within the workspace
        if (np.sqrt(point[0]^2 + point[1]^2 + point[2]^2) > self.arm.max_reach):
            # Set the in_workspace flag
            in_workspace = False

        # Return the in_workspace flag
        return in_workspace

    # Define the function to create a new node
    def new_node(self, point):
        # Initialize the new node
        new_node = point + [np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)]

        # Compare the new node to the obstacles
        if (~self.in_obstacle(new_node)):
            return new_node
        else:
            return None
    
    # Define the function to check if we are interfering with ourselves
    # def self_interference(self, point):

    # TODO: If the functions below can be interpreted, it may be useful for path smoothing
    # # Define the function to find the nearest node
    # def nearest_node(self, point):
    #     # Initialize the nearest node
    #     nearest_node = self.tree[0]

    #     # Iterate through the tree
    #     for node in self.tree:
    #         # Check if the distance to the point is less than the distance to the nearest node
    #         if (math.sqrt((point[0] - node[0])**2 + (point[1] - node[1])**2) <
    #             math.sqrt((point[0] - nearest_node[0])**2 + (point[1] - nearest_node[1])**2)):
    #             # Set the nearest node
    #             nearest_node = node

    #     # Return the nearest node
    #     return nearest_node

    # # Define the function to find the new point
    # def new_point(self, nearest_node, point):
    #     # Initialize the new point
    #     new_point = [0, 0]

    #     # Check if the distance to the point is less than the stepsize
    #     if (math.sqrt((point[0] - nearest_node[0])**2 + (point[1] - nearest_node[1])**2) < self.stepsize):
    #         # Set the new point
    #         new_point = point
    #     else:
    #         # Calculate the angle to the point
    #         angle = math.atan2(point[1] - nearest_node[1], point[0] - nearest_node[0])

    #         # Set the new point
    #         new_point = [nearest_node[0] + self.stepsize*math.cos(angle),
    #                      nearest_node[1] + self.stepsize*math.sin(angle)]
