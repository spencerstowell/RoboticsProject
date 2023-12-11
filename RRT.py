# Description: This file defines a class for Rapidly Exploring Random Trees (RRTs)
# specifically for the purpose of path planning for a robot arm.

## TODO: Add a function to smooth the path
## TODO: Add functionality to only propagate new nodes from 'end' nodes

# Import needed libraries
import numpy as np
import random
import matplotlib.pyplot as plt
import time
from visualization import VizScene

# Define the class for the RRT algorithm
class RRT:
    # Define the initialization function
    def __init__(self, start, goal, obstacles, stepsize, max_iter, children, arm):
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
        children - the number of 'children' nodes to generate for each 'parent' node \n
        arm - a robot arm object with all parameters for calculating intermediate points
        """

        self.arm = arm

        # Initialize the start and goal points
        self.start = start
        # print(start)
        self.goal = goal

        # Initialize the obstacles
        self.obstacles = obstacles

        # Initialize the stepsize and max iterations
        self.stepsize = stepsize
        self.max_iter = max_iter
        self.children = children

        # Initialize the tree
        self.tree = [start]

        # Initialize the path
        self.path = []

        # Initialize the path_found flag
        self.path_found = False

    # Define the function to run the RRT algorithm
    def run(self):
        '''
        This function runs the RRT algorithm
        '''
        # Initialize the iteration counter
        i = 0

        self.goal_node = []
        self.goal_iterations = 0

        # Initialize the number of 'children' nodes to generate for each 'parent'
        n = self.children

        # Run the algorithm until the max iterations is reached or the path is found
        while (i < self.max_iter / n) and (not self.path_found):
            
            for _ in range(0, n):
                # Initialize the new node
                new_node = []

                # Generate a new node at the selected location in the tree until it is not returned as None
                while (new_node == []):
                    new_node, reached_goal = self.new_node(self.tree[i], self.stepsize)

                # Add the new node to the tree
                self.tree.append(new_node)

                # Check if the new node has reached the goal & is the first to do so
                if (reached_goal and self.goal_node == []):
                    # Set the path_found flag
                    self.path_found = True
                    # Check at what point the goal was located
                    self.goal_iterations = 3*i+1
                    # Save the goal node for path retracing later
                    self.goal_node = new_node
                    # Break out of the 'for' loop
                    break

            # Increment the iteration counter
            i += 1
        
        # Retrace the path
        self.retrace_path()
        

    # Define the function to retrace the path (if it exists)
    def retrace_path(self):
        '''
        This function retraces the path from the goal to the start
        '''
        # Check if the path has been found
        if (self.path_found):
            # Initialize the current node
            current_node = self.goal_node

            # Initialize the path object
            self.path = []

            # Iterate through the tree
            while (current_node != self.start):
                # Add the current node to the path
                self.path.append(current_node)

                # Set the current node to the parent node (located in the last index of the current node)
                current_node = current_node[-1]

            # Add the start node to the path
            self.path.append(self.start)

            # Reverse the order of the nodes to go from start to finish
            self.path.reverse()

            print("Path found in " + str(self.goal_iterations) + " iterations")

            self.smoothed_path = []

            print("Smoothing the path")
            while (self.smoothed_path != self.smooth_path()):
                pass

        else:
            print("No path found")
    
    # Define a function to smooth the path
    def smooth_path(self):
        '''
        This function searches through all the nodes in the tree to identify which nodes
        on it's branch have a 'line of sight' to other nodes farther down. If
        there is, it will replace the subsequent node with the new one to 'smooth' the path. \n
        This function technically also returns the smoothed path, but only for the purpose of
        checking the while loop in the retrace_path function.
        '''

        # Initialize the current node as the goal node
        current_node = self.goal_node

        # Initialize the path object & include the goal node
        self.smoothed_path = [self.goal_node]

        # Initialize the 'keep' node as the goal node
        keep_node = [self.goal_node]

        # Start iterating through nodes
        while (current_node != self.start):
            next_node = current_node[-1]

            # While the current node is in line of sight of the next node
            while(self.check_line_of_sight(current_node, next_node)):
                # Move the 'next node' to the one after it & store the one we just checked
                keep_node = next_node
                next_node = next_node[-1]
                    
                # Check if we've reached the start node
                if (keep_node == self.start):
                    # Break out of the while loop
                    break
                
            # Add the 'keep' node to the path
            self.smoothed_path.append(keep_node)

            # Set the new 'current' node to the 'keep' node
            current_node = keep_node

        # Reverse the order of the nodes to go from start to finish
        self.smoothed_path.reverse()

        return self.smoothed_path
        
    def check_line_of_sight(self, node1, node2):
        '''
        This function checks if there is a clear line of sight between two nodes. \n
        (i.e. there are no obstacles in the way) \n
        node1 - the first node \n
        node2 - the second node \n

        This returns a boolean flag indicating whether there is a clear line of sight between the two nodes.
        '''

        # Initialize the line_of_sight flag
        line_of_sight = True

        # Get the cartesian coordinates of the two nodes
        node1 = node1[0]
        node2 = node2[0]

        # Create a line between the two nodes with 100 points
        x = np.linspace(node1[0], node2[0], 100)
        y = np.linspace(node1[1], node2[1], 100)
        z = np.linspace(node1[2], node2[2], 100)

        # Iterate through the obstacles
        for obs in self.obstacles:
            for i in range(0, 100):
                # Check if the line intersects with the obstacle
                if (np.sqrt((x[i] - obs[0])**2 + (y[i] - obs[1])**2 + (z[i] - obs[2])**2) < obs[3]):
                    # Set the line_of_sight flag
                    line_of_sight = False
        
        # Return the line_of_sight flag
        return line_of_sight

    def plot(self):
        # Plot the obstacles & all nodes in the tree on a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the goal as a wireframe sphere
        r = self.goal[3]
        u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
        x = np.cos(u) * np.sin(v) * r + self.goal[0]
        y = np.sin(u) * np.sin(v) * r  + self.goal[1]
        z = np.cos(v) * r + self.goal[2]
        ax.plot_wireframe(x, y, z, color="g")
        
        # Plot the obstacles as wireframe spheres
        for i in self.obstacles:
            obs = i
            r = obs[3]
            u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
            x = np.cos(u) * np.sin(v) * r + obs[0]
            y = np.sin(u) * np.sin(v) * r + obs[1]
            z = np.cos(v) * r + obs[2]
            ax.plot_wireframe(x, y, z, color="r")
            # ax.plot_surface(x, y, z, color="r")

        r = self.arm.max_reach
        u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
        x = np.cos(u) * np.sin(v) * r
        y = np.sin(u) * np.sin(v) * r
        z = np.abs(np.cos(v)) * r
        ax.plot_wireframe(x, y, z, color="b")
        
        # Plot all nodes in the tree
        # if (not self.path_found):
        for node in self.tree:
            ax.scatter(node[0][0], node[0][1], node[0][2], c='b', marker='.')

        # Set the plot labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    def plot_path(self):
        # Plot the obstacles & all nodes in the tree on a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        path_x = []
        path_y = []
        path_z = []
        smooth_x = []
        smooth_y = []
        smooth_z = []

        # Plot the goal as a wireframe sphere
        r = self.goal[3]
        u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
        x = np.cos(u) * np.sin(v) * r + self.goal[0]
        y = np.sin(u) * np.sin(v) * r  + self.goal[1]
        z = np.cos(v) * r + self.goal[2]
        ax.plot_wireframe(x, y, z, color="g")
        
        # Plot the obstacles as wireframe spheres
        for i in self.obstacles:
            obs = i
            r = obs[3]
            u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
            x = np.cos(u) * np.sin(v) * r + obs[0]
            y = np.sin(u) * np.sin(v) * r + obs[1]
            z = np.cos(v) * r + obs[2]
            ax.plot_wireframe(x, y, z, color="r")
            # ax.plot_surface(x, y, z, color="r")

        r = self.arm.max_reach
        u, v = np.mgrid[0:2 * np.pi:30j, 0:np.pi:20j]
        x = np.cos(u) * np.sin(v) * r
        y = np.sin(u) * np.sin(v) * r
        z = np.abs(np.cos(v)) * r
        ax.plot_wireframe(x, y, z, color="b")

        # Arrange the data points for the path
        for node in self.path:
            path_x.append([node[0][0]])
            path_y.append([node[0][1]])
            path_z.append([node[0][2]])
        # plot the sequence of nodes that constitutes the path
        ax.plot(path_x, path_y, path_z, c='g', marker='o')

        # Arrange the data points for the smoothed path
        for node in self.smoothed_path:
            smooth_x.append([node[0][0]])
            smooth_y.append([node[0][1]])
            smooth_z.append([node[0][2]])
        # plot the sequence of nodes that constitutes the path
        ax.plot(smooth_x, smooth_y, smooth_z, c='y', marker='o')

        # Save the path points for later use in robot arm simulation
        #self.smooth_points = [path_x, path_y, path_z]
        self.smooth_points = [smooth_x, smooth_y, smooth_z]
        # Set the plot labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    # Define the function to create a new node
    def new_node(self, point, stepsize):
        '''
        This function accepts a starting point and a stepsize to use in
        generating a random new node. \n
        point - a 3D cartesian coordinate point (X, Y, Z) \n
        stepsize - a scalar value

        The function returns the newly generated node if it meets the following criteria: \n
        1. The new node is not in an obstacle \n
        2. The new node is in the workspace \n
        The probability the node is returned, provided it meets the above criteria, is
        dependent on whether it is closer to the goal than the current node.

        This also checks if the new node has reached the goal & returns a boolean flag accordingly \n
        \tex: new_node, reached_goal = self.new_node(point, stepsize)
        '''

        while(1):
            # Initialize the location of the new node
            new_node = [point[0][0] + np.random.uniform(-stepsize, stepsize),
                        point[0][1] + np.random.uniform(-stepsize, stepsize),
                        point[0][2] + np.random.uniform(-stepsize, stepsize)]
            
            # Append the starting node to the end of the new node
            new_node = [new_node[:], point]

            # Generate a random number between 0 and 1 that will be used to determine if the new node is added to the tree
            random_num = np.random.uniform(0, 1)

            threshold = 0.15

            # Check if the new node has reached the goal
            reached_goal = self.reached_goal(new_node)

            # Compare the new node to the obstacles
            if not self.in_obstacle(new_node) and self.in_workspace(new_node) and self.check_line_of_sight(point, new_node):
                
                # Check if the new node is closer to the goal than the current node
                if (self.check_direction(point, new_node) and random_num >= threshold):
                    # Return the new node
                    return new_node, reached_goal
                
                # If the new node is farther away from the goal than the current node, there is still a 20% chance
                # that it will be added to the tree
                elif (not self.check_direction(point, new_node) and random_num < threshold):
                    # Return the new node
                    return new_node, reached_goal

    # Define the function to check if a point is in an obstacle
    def in_obstacle(self, point):
        '''
        This function checks if a newly generated node is within the radius of any obstacles
        point - List of X, Y, Z location in space

        ex: in_obstacle = self.in_obstacle(point)
        '''
        # Initialize the in_obstacle flag
        in_obstacle = False

        # Iterate through the obstacles
        for i in self.obstacles:
            obs = i
            # Check if the point is in the obstacle by comparingthe magnitude of:
            # the distance of the end effector to the center of the obstacle
            # to:
            # the radius of the obstacle
            # This is inherently designed to work with spherical obstacles

            if (np.sqrt((point[0][0] - obs[0])**2 + (point[0][1] - obs[1])**2 + (point[0][2] - obs[2])**2) < obs[3]):
                # Set the in_obstacle flag
                in_obstacle = True

        # Return the in_obstacle flag
        return in_obstacle
    
    def in_workspace(self, point):
        '''
        This function checks if a newly generated node is within the workspace of the robot
        point - List of X, Y, Z location in space

        ex: in_workspace = self.in_workspace(point)
        '''
        # Initialize the in_workspace flag
        in_workspace = True
        # Check if the point is within the workspace
        if (np.sqrt(point[0][0]**2 + point[0][1]**2 + point[0][2]**2) > self.arm.max_reach):
            # Set the in_workspace flag
            in_workspace = False

        elif (point[0][2] < 0):
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

        ex: is_closer = self.check_direction(node, new_node)
        '''
        # Initialize the is_closer flag
        is_closer = True

        # Calculate the distance from the current node to the goal
        current_node_distance = np.sqrt((self.goal[0] - node[0][0])**2
                                        + (self.goal[1] - node[0][1])**2
                                        + (self.goal[2] - node[0][2])**2)
        
        # Calculate the distance from the new node to the goal
        new_node_distance = np.sqrt((self.goal[0] - new_node[0][0])**2
                                    + (self.goal[1] - new_node[0][1])**2
                                    + (self.goal[2] - new_node[0][2])**2)

        # Check if the new node is closer to the goal than the current node
        if (new_node_distance > current_node_distance):
            # Set the check_direction flag
            is_closer = False

        if (new_node_distance < self.stepsize):
            self.stepsize = new_node_distance

        # Return the check_direction flag
        return is_closer

    def reached_goal(self, point):
        '''
        This function checks if the end effector has reached the goal
        point - the current end effector position

        ex: reached_goal = self.reached_goal(point)
        '''
        # Initialize the reached_goal flag
        reached_goal = False

        # print(point[0][0])
        # print(self.goal)
        # Check if the end effector is within the goal radius
        if (np.sqrt((self.goal[0] - point[0][0])**2 + (self.goal[1] - point[0][1])**2 + (self.goal[2] - point[0][2])**2) < self.goal[3]):
            # Set the reached_goal flag
            reached_goal = True
        # Return the reached_goal flag
        return reached_goal

# Test code for the RRT class
if __name__ == "__main__":
    import kinematics as kin

    # Define the DH parameters
    Dh = [[0, 0, 0., np.pi/2.0],
            [0, 0, 1, 0], 
            [0, 0, 1, 0]]
    
    arm = kin.SerialArm(Dh)
    # Define the start point from a given arm configuration
    start = [arm.fk(q=[0, 0, 0])[:3,3].tolist(), [None]]

    # Define the desired goal point
    goal = [-1.0, 0.0, 1.25, 0.15]

    # Define the obstacles
    obstacles = [[1, 0.5, 0.75, 0.75], [-0.5, -0.5, 1.0, 0.5]]


    

    # Define the stepsize and max iterations
    stepsize = arm.max_reach/4
    max_iter = 200000
    children = 2

    # Initialize the RRT
    print("Initializing RRT")
    rrt = RRT(start, goal, obstacles, stepsize, max_iter, children, arm)

    # Run the RRT algorithm
    print("Running RRT")
    rrt.run()

    # Plot the results
    print("Plotting results")
    # rrt.plot()
    rrt.plot_path()

    # arm.ik_position(rrt.plot_points[0][-1], rrt.plot_points[1][-1], rrt.plot_points[2][-1])
    #%%
    # Interpolate between striaght paths
    print("Interpolating between straight paths")
    smooth_points_temp = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(len(rrt.smooth_points[0])-2):
        # Get the cartesian coordinates of the two nodes
        node1 = [rrt.smooth_points[0][i],rrt.smooth_points[1][i],rrt.smooth_points[2][i]]
        node2 = [rrt.smooth_points[0][i+1],rrt.smooth_points[1][i+1],rrt.smooth_points[2][i+1]]

        # Create a line between the two nodes with 100 points
        x = np.linspace(node1[0], node2[0], 10)
        y = np.linspace(node1[1], node2[1], 10)
        z = np.linspace(node1[2], node2[2], 10)

        # save as vector points
        smooth_points_temp[0][i] = x
        smooth_points_temp[1][i] = y
        smooth_points_temp[2][i] = z

    print("done")




    # Get the joint angles for the smooth path points and interpolate values in between
    print("Calculating IK Joint angles")
    q = []
    for i in range(len(rrt.smooth_points[0])):
        target_temp = [rrt.smooth_points[0][i][0], rrt.smooth_points[1][i][0], rrt.smooth_points[2][i][0]]
        q_temp = (arm.ik_position(target = target_temp,q0 = None, method = 'J_T',force = True, tol = 1e-4,K = np.eye(3),kd =0.001, max_iter = 1000))
        q.append(q_temp[0])
        #q0 = q_temp[0]
    # add goal as final point
    q.append(arm.ik_position(target = goal[0:3],q0 = None, method = 'J_T',force = True, tol = 1e-4,K = np.eye(3),kd =0.001, max_iter = 1000)[0])
    q_points = q




    #%% plot the joint paths
    q = np.array(q)
    plt.figure()
    plt.plot(q[:,0])
    plt.plot(q[:,1])
    plt.plot(q[:,2])
    plt.title('Joint Paths')
    plt.xlabel('Time')
    plt.ylabel('Joint Angle')
    plt.legend(['Joint 1', 'Joint 2', 'Joint 3'])
    plt.show()


    #%% visualize the Robot arm in Rvis
    # making a visualization
    viz = VizScene()

    # run time
    time_to_run = 7
    refresh_rate = len(q)/time_to_run

    # add arm
    viz.add_arm(arm, draw_frames=True)
    qs = q[0]

    # add obastacles 
    viz.add_obstacle(obstacles[0][0:3], rad = obstacles[0][3])
    viz.add_obstacle(obstacles[1][0:3], rad = obstacles[1][3])
    # add goal
    viz.add_marker(goal[0:3])
    viz.add_frame(arm.fk(q[len(q)-1]), label ="goal")

    # add start
    viz.add_marker(start[0])
    viz.add_frame(arm.fk(q[0]), label ="start")

    # add major points 
    for i in range(len(q_points)):
        viz.add_marker(arm.fk(q_points[i])[:3,3].tolist())
    
    ## add points inbetween
    #for i in range(len(q)):
    #    viz.add_marker(arm.fk(q[i])[:3,3].tolist())
    

    # loop through and update the arm
    for i in range(len(q)):
        viz.update(qs=[q[i]])
        time.sleep(1.0/refresh_rate)
    
    viz.hold()





    viz.close_viz()




# %%
