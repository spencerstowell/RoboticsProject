# I need to write a ROS script that will take input from the user for
# a desired position, with a number/location of obstacles, and then
# run an RRT algorithm to find a path to the desired location. This will then
# publish the sequence of angles to the ROS network. A different node will subscribe to this
# topic and use the input to move the robot, based on the "cheapstepper" arduino
# library. This will be done in a separate script, as I don't want to have to
# re-run the RRT algorithm every time I want to move the robot.

# Can you help me with this? I'm not sure how to initialize the ROS node
# and then have it wait for input from the user.

# Import needed libraries
import rospy
import numpy as np
import math
import random
import matplotlib.pyplot as plt

# Import needed messages
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

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
            Future idea - include another list parameter specifying spherical or cuboid obstacles
        stepsize - the stepsize for the RRT algorithm
        max_iter - the maximum number of iterations for the RRT algorithm
        arm - a robot arm object with all parameters for calculating intermediate points
        """

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
            # Check if the point is in the obstacle
            if (np.sqrt((point[0] - obs[i,0])^2 + (point[1] - obs[i,1])^2 + (point[2] - obs[i,2])^2) < obs[i,3]):
                # Set the in_obstacle flag
                in_obstacle = True

        # Return the in_obstacle flag
        return in_obstacle

    # Define the function to check if we are interfering with ourselves
    # def self_interference(self, point):

    # Define the function to find the nearest node
    def nearest_node(self, point):
        # Initialize the nearest node
        nearest_node = self.tree[0]

        # Iterate through the tree
        for node in self.tree:
            # Check if the distance to the point is less than the distance to the nearest node
            if (math.sqrt((point[0] - node[0])**2 + (point[1] - node[1])**2) <
                math.sqrt((point[0] - nearest_node[0])**2 + (point[1] - nearest_node[1])**2)):
                # Set the nearest node
                nearest_node = node

        # Return the nearest node
        return nearest_node

    # Define the function to find the new point
    def new_point(self, nearest_node, point):
        # Initialize the new point
        new_point = [0, 0]

        # Check if the distance to the point is less than the stepsize
        if (math.sqrt((point[0] - nearest_node[0])**2 + (point[1] - nearest_node[1])**2) < self.stepsize):
            # Set the new point
            new_point = point
        else:
            # Calculate the angle to the point
            angle = math.atan2(point[1] - nearest_node[1], point[0] - nearest_node[0])

            # Set the new point
            new_point = [nearest_node[0] + self.stepsize*math.cos(angle),
                         nearest_node[1] + self.stepsize*math.sin(angle)]
            
