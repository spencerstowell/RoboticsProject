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
        Accepts arguments start, goal, obstacles, stepsize, and max_iter \n
        start - the starting point of the end effector in 3D cartesian coordinates \n
        goal - the goal point of the end effector in 3D cartesian coordinates with a radius \n
            \tex - [goal_x, goal_y, goal_z, radius] \n
        obstacles - a list of obstacles, where each obstacle is defined in 3D cartesian coordinates with a radius \n
            \tex - [[x1, y1, z1, radius1], [x2, y2, z2, radius2], ...] \n
            \tFuture idea - include another list parameter specifying spherical vs boxy obstacles \n
        stepsize - the stepsize for the RRT algorithm (scalar value) \n
        max_iter - the maximum number of iterations for the RRT algorithm (scalar value) \n
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

    # Define the function to create a new node
    def new_node(self, point, stepsize):
        '''
        This function accepts a starting point and a stepsize to use in
        generating a random new node. \n
        point - a 3D cartesian coordinate point (X, Y, Z) \n
        stepsize - a scalar value
        '''
        # Initialize the new node
        new_node = point + [np.random.uniform(-stepsize, stepsize),
                            np.random.uniform(-stepsize, stepsize),
                            np.random.uniform(-stepsize, stepsize)]

        # Generate a random number between 0 and 1 that will be used to determine if the new node is added to the tree
        random_num = np.random.uniform(0, 1)

        # Check if the new node has reached the goal
        goal = self.reached_goal(new_node)

        # Compare the new node to the obstacles
        if (~self.in_obstacle(new_node) and self.in_workspace(new_node)):
            
            # Check if the new node is closer to the goal than the current node
            if (self.check_direction(point, new_node) and random_num >= 0.2):
                # Return the new node
                return new_node, goal
            
            # If the new node is farther away from the goal than the current node, there is still a 20% chance
            # that it will be added to the tree
            elif (~self.check_direction(point, new_node) and random_num < 0.2):
                # Return the new node
                return new_node, goal
        else:
            return None

    # Define the function to check if a point is in an obstacle
    def in_obstacle(self, point):
        '''
        This function checks if a newly generated node is within the radius of any obstacles
        point - List of X, Y, Z location in space
        '''
        # Initialize the in_obstacle flag
        in_obstacle = False

        # Iterate through the obstacles
        for i in self.obstacles:
            obs = self.obstacles[i]
            # Check if the point is in the obstacle by comparingthe magnitude of:
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
        '''
        This function checks if a newly generated node is within the workspace of the robot
        point - List of X, Y, Z location in space
        '''
        # Initialize the in_workspace flag
        in_workspace = True
        # Check if the point is within the workspace
        if (np.sqrt(point[0]^2 + point[1]^2 + point[2]^2) > self.arm.max_reach):
            # Set the in_workspace flag
            in_workspace = False

        # Return the in_workspace flag
        return in_workspace
    
    # Define the function to check if the new node is closer to the goal than the current node
    def check_direction(self, node, new_node):
        '''
        This function checks if the new node generated is closer to the goal
        than the starting point. \n
        node - the starting point that new_node is generated from (3D cartesian coordinate point) \n
        new_node - the new node we're comparing (3D cartesian coordinate point)
        '''
        # Initialize the is_closer flag
        is_closer = True

        # Calculate the distance from the current node to the goal
        current_node_distance = np.sqrt((self.goal[0] - node[0])**2
                                        + (self.goal[1] - node[1])**2
                                        + (self.goal[2] - node[2])**2)
        
        # Calculate the distance from the new node to the goal
        new_node_distance = np.sqrt((self.goal[0] - new_node[0])**2
                                    + (self.goal[1] - new_node[1])**2
                                    + (self.goal[2] - new_node[2])**2)

        # Check if the new node is closer to the goal than the current node
        if (new_node_distance > current_node_distance):
            # Set the check_direction flag
            is_closer = False

        # Return the check_direction flag
        return is_closer

    def reached_goal(self, point):
        '''
        This function checks if the end effector has reached the goal
        point - the current end effector position
        '''
        # Initialize the reached_goal flag
        reached_goal = False

        # Check if the end effector is within the goal radius
        if (np.sqrt((self.goal[0] - point[0])**2 + (self.goal[1] - point[1])**2 + (self.goal[2] - point[2])**2) < self.goal[3]):
            # Set the reached_goal flag
            reached_goal = True

        # Return the reached_goal flag
        return reached_goal
    

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
