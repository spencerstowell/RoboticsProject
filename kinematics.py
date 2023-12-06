"""
Kinematics Module - Contains code for:
- Forward Kinematics, from a set of DH parameters to a serial linkage arm with callable forward kinematics
- Inverse Kinematics
- Jacobian

John Morrell, Jan 26 2022
Tarnarmour@gmail.com

modified by: 
Marc Killpack, Sept 21, 2022 and Sept 21, 2023
"""

from transforms import *
import numpy as np
from utility import skew
eye = np.eye(4)
pi = np.pi


# this is a convenience class that makes it easy to define a function that calculates "A_i(q)", given the
# DH parameters for link and joint "i" only. 
class dh2AFunc:
    """
    A = dh2AFunc(dh, joint_type="r")
    Description:
    Accepts a list of 4 dh parameters corresponding to the transformation for one link 
    and returns a function "f" that will generate a homogeneous transform "A" given 
    "q" as an input. A represents the transform from link i-1 to link i. This follows
    the "standard" DH convention. 

    Parameters:
    dh - 1 x 4 list from dh parameter table for one transform from link i-1 to link i,
    in the order [theta d a alpha] - THIS IS NOT THE CONVENTION IN THE BOOK!!! But it is the order of operations. 

    Returns:
    f(q) - a function that can be used to generate a 4x4 numpy matrix representing the homogeneous transform 
        from one link to the next
    """
    def __init__(self, dh, jt):

        # if joint is revolute implement correct equations here:
        if jt == 'r':
            # although A(q) is only a function of "q", the dh parameters are available to these next functions 
            # because they are passed into the "init" function above. 

            def A(q): 
                # See eq. (2.52), pg. 64
                # TODO - complete code that defines the "A" or "T" homogenous matrix for a given set of DH parameters. 
                # Do this in terms of the variables "dh" and "q" (so that one of the entries in your dh list or array
                # will need to be added to q).
                T = se3(rotz(q+dh[0]),[0, 0, dh[1]]) @ se3(rotx(dh[3]),[dh[2], 0, 0])
                return T


        # if joint is prismatic implement correct equations here:
        else:
            def A(q):
                # See eq. (2.52), pg. 64
                # TODO - complete code that defines the "A" or "T" homogenous matrix for a given set of DH parameters. 
                # Do this in terms of the variables "dh" and "q" (so that one of the entries in your dh list or array
                # will need to be added to q).
                T = se3(rotz(dh[0]),[0, 0, dh[1]+q]) @ se3(rotx(dh[3]),[dh[2], 0, 0])
                return T


        self.A = A


class SerialArm:
    """
    SerialArm - A class designed to represent a serial link robot arm

    SerialArms have frames 0 to n defined, with frame 0 located at the first joint and aligned with the robot body
    frame, and frame n located at the end of link n.

    """


    def __init__(self, dh_params, jt=None, base=eye, tip=eye, joint_limits=None):
        """
        arm = SerialArm(dh_params, joint_type, base=I, tip=I, radians=True, joint_limits=None)
        :param dh: n length list where each entry in list is another list of length 4, representing dh parameters, [theta d a alpha]
        :param jt: n length list of strings, 'r' for revolute joint and 'p' for prismatic joint
        :param base: 4x4 numpy array representing SE3 transform from world or inertial frame to frame 0
        :param tip: 4x4 numpy array representing SE3 transform from frame n to tool frame or tip of robot
        :param joint_limits: 2 length list of n length lists, holding first negative joint limit then positive, none for
        not implemented
        """
        
        self.dh = dh_params
        self.n = len(dh_params)

        self.reach = 0
        for i in range(self.n):
            self.reach += np.sqrt(self.dh[i][0]**2 + self.dh[i][2]**2)

        self.max_reach = 0.0
        for dh in self.dh:
            self.max_reach += norm(np.array([dh[0], dh[2]]))

        self.dh = dh_params
        self.n = len(dh_params)

        # we will use this list to store the A matrices for each set/row of DH parameters. 
        self.transforms = []

        # assigning a joint type
        if jt is None:
            self.jt = ['r'] * self.n
        else:
            self.jt = jt
            if len(self.jt) != self.n:
                print("WARNING! Joint Type list does not have the same size as dh param list!")
                return None

        # using the code we wrote above to generate the function A(q) for each set of DH parameters
        for i in range(self.n):
            # TODO use the class definition above (dh2AFunc), and the dh parameters and joint type to
            # make a function and then append that function to the "transforms" list. 
            f = dh2AFunc(self.dh[i],self.jt[i])
            self.transforms.append(f.A)


        # assigning the base, and tip transforms that will be added to the default DH transformations.
        self.base = base
        self.tip = tip
        self.qlim = joint_limits


    def fk(self, q, index=None, base=False, tip=False):
        """
            T = arm.fk(q, index=None, base=False, tip=False)
            Description: 
                Returns the transform from a specified frame to another given a 
                set of joint inputs q and the index of joints

            Parameters:
                q - list or iterable of floats which represent the joint positions
                index - integer or list of two integers. If a list of two integers, the first integer represents the starting JOINT 
                    (with 0 as the first joint and n as the last joint) and the second integer represents the ending FRAME
                    If one integer is given only, then the integer represents the ending Frame and the FK is calculated as starting from 
                    the first joint
                base - bool, if True then if index starts from 0 the base transform will also be included
                tip - bool, if true and if the index ends at the nth frame then the tool transform will be included
            
            Returns:
                T - the 4 x 4 homogeneous transform from frames determined from "index" variable
        """

        ###############################################################################################
        # the following lines of code are data type and error checking. You don't need to understand
        # all of it, but it is helpful to keep. 

        if not hasattr(q, '__getitem__'):
            q = [q]

        if len(q) != self.n:
            print("WARNING: q (input angle) not the same size as number of links!")
            return None

        if isinstance(index, (list, tuple)):
            start_frame = index[0]
            end_frame = index[1]
        elif index == None:
            start_frame = 0
            end_frame = self.n
        else:
            start_frame = 0
            if index < 0:
                print("WARNING: Index less than 0!")
                print(f"Index: {index}")
                return None
            end_frame = index

        if end_frame > self.n:
            print("WARNING: Ending index greater than number of joints!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame < 0:
            print("WARNING: Starting index less than 0!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        if start_frame > end_frame:
            print("WARNING: starting frame must be less than ending frame!")
            print(f"Starting frame: {start_frame}  Ending frame: {end_frame}")
            return None
        ###############################################################################################        
        ###############################################################################################


        # TODO - Write code to calculate the total homogeneous transform "T" based on variables stored
        # in "base", "tip", "start_frame", and "end_frame". Look at the function definition if you are 
        # unsure about the role of each of these variables. This is mostly easily done with some if/else 
        # statements and a "for" loop to add the effect of each subsequent A_i(q_i). But you can 
        # organize the code any way you like.  
        T = eye #defined above as eye = np.eye(4)

        if base:
            T = T @ self.base

        for i in range(start_frame, end_frame):
            T = T @ self.transforms[i](q[i])
        
        if tip and end_frame == self.n:
            T = T @ self.tip
        return T


    # You don't need to touch this function, but it is helpful to be able to "print" a description about
    # the robot that you make.
    def __str__(self):
        """
            This function just provides a nice interface for printing information about the arm. 
            If we call "print(arm)" on an SerialArm object "arm", then this function gets called.
            See example in "main" below. 
        """
        dh_string = """DH PARAMS\n"""
        dh_string += """theta\t|\td\t|\ta\t|\talpha\t|\ttype\n"""
        dh_string += """---------------------------------------\n"""
        for i in range(self.n):
            dh_string += f"{self.dh[i][0]}\t|\t{self.dh[i][1]}\t|\t{self.dh[i][2]}\t|\t{self.dh[i][3]}\t|\t{self.jt[i]}\n"
        return "Serial Arm\n" + dh_string
    
    ## copy this function into the main SerialArm class and complete the TODO below
    def jacob(self, q, index=None, base=False, tip=False):
        """
        J = arm.jacob(q)
        Description: 
        Returns the geometric jacobian for the frame defined by "index", which corresponds
        to a frame on the arm, with the arm in a given configuration defined by "q"

        Parameters:
        q - list or numpy array of joint positions
        index - integer, which joint frame at which to calculate the Jacobian

        Returns:
        J - numpy matrix 6xN, geometric jacobian of the robot arm
        """


        if index is None:
            index = self.n
        elif index > self.n:
            print("WARNING: Index greater than number of joints!")
            print(f"Index: {index}")

        # TODO - start by declaring a zero matrix that is the correct size for the Jacobian

        #In this case, we know the number of rows will always be six, and the number of columns
        #will be equal to the number of joints being considered, provided by the value contained
        #in 'index'
        J = np.zeros((6, self.n))

        # TODO - find the current position of the point of interest (usually origin of frame "n") 
        # using your fk function. This will likely require additional intermediate variables than 
        # what is shown here. 

        # Pull out the coordinates of the point of interest from T (4x4 array)
        # Located in the first three rows (indices 0, 1, & 2) of the last column of T (index 3)
        # Provides a 3x1 array

        pe = self.fk(q, index, base=base, tip=tip)[:3, 3]


        # TODO - calculate all the necessary values using your "fk" function, and fill every column
        # of the jacobian using this "for" loop. Functions like "np.cross" may also be useful.

        # Initial Thoughts:
        # Necessary values include:
        # The coordinates of each origin (similar to how we located the point of interest 'pe')
        # The orientation of our z-axis for each origin (first three rows of the 2nd-to-last column of T)

        for i in range(index):
            #should 'base'be included in the fk call?
            #should 'tip' be included in the last fk call?
            Ti = self.fk(q, i, base, tip)
            # Explanation of Ti:
            # we're calculating the forward kinematics (homogeneous transformation matrix T)
            # from the base frame to the i-th frame. As such, every joint parameter from base to i
            # within 'q' needs to be passed in. The index is simply the current value of i

            Origin_i = Ti[:3, 3]    #Indices corresponding to the location of the i-th frame
            Orientation_i = Ti[:3, 2]   #Indices corresponding to the orientation of the z-axis
            
            # check if joint is revolute
            if self.jt[i] == 'r':
                J[:3,i] = np.cross(Orientation_i, (pe - Origin_i))
                J[3:,i] = Orientation_i

            # if not assume joint is prismatic
            else:
                J[:3,i] = Orientation_i
                J[3:,i] = np.array([[0],[0],[0]]).flatten()

        return J
    
    def ik_position(self, target, q0=None, method='J_T', force=True, tol=1e-4, K=None, kd=0.001, max_iter=100):
        """
        (qf, ef, iter, reached_max_iter, status_msg) = arm.ik2(target, q0=None, method='jt', force=False, tol=1e-6, K=None)
        Description:
            Returns a solution to the inverse kinematics problem finding
            joint angles corresponding to the position (x y z coords) of target

        Args:
            target: 3x1 numpy array that defines the target location. 

            q0: length of initial joint coordinates, defaults to q=0 (which is
            often a singularity - other starting positions are recommended)

            method: String describing which IK algorithm to use. Options include:
                - 'pinv': damped pseudo-inverse solution, qdot = J_dag * e * dt, where
                J_dag = J.T * (J * J.T + kd**2)^-1
                - 'J_T': jacobian transpose method, qdot = J.T * K * e

            force: Boolean, if True will attempt to solve even if a naive reach check
            determines the target to be outside the reach of the arm

            tol: float, tolerance in the norm of the error in pose used as termination criteria for while loop

            K: 3x3 numpy matrix. For both pinv and J_T, K is the positive definite gain matrix used for both. 

            kd: is a scalar used in the pinv method to make sure the matrix is invertible. 

            max_iter: maximum attempts before giving up.

        Returns:
            qf: 6x1 numpy matrix of final joint values. If IK fails to converge the last set
            of joint angles is still returned

            ef: 3x1 numpy vector of the final error

            count: int, number of iterations

            flag: bool, "true" indicates successful IK solution and "false" unsuccessful

            status_msg: A string that may be useful to understanding why it failed. 
        """
        # Fill in q if none given, and convert to numpy array 
        if isinstance(q0, np.ndarray):
            q = q0
        elif q0 == None:
            q = np.array([0.0]*self.n)
        else:
            q = np.array(q0)

        # initializing some variables in case checks below don't work
        error = None
        count = 0

        # Try basic check for if the target is in the workspace.
        # Maximum length of the arm is sum(sqrt(d_i^2 + a_i^2)), distance to target is norm(A_t)
        maximum_reach = 0
        for i in range(self.n):  # Add max length of each link
            maximum_reach = maximum_reach + np.sqrt(self.dh[i][1] ** 2 + self.dh[i][2] ** 2)

        pt = target  # Find distance to target
        target_distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2)

        if target_distance > maximum_reach and not force:
            print("WARNING: Target outside of reachable workspace!")
            return q, error, count, False, "Failed: Out of workspace"
        else:
            if target_distance > maximum_reach:
                print("Target out of workspace, but finding closest solution anyway")
            else:
                print("Target passes naive reach test, distance is {:.1} and max reach is {:.1}".format(
                    float(target_distance), float(maximum_reach)))

        if not isinstance(K, np.ndarray):
            return q, error, count, False,  "No gain matrix 'K' provided"



        # you may want to define some functions here to help with operations that you will 
        # perform repeatedly in the while loop below. Alternatively, you can also just define 
        # them as class functions and use them as self.<function_name>.

        # for example:

        #This fn only gets the positional error, and doesn't consider orientation
        def get_error(q):
            #Update the current position with the most recent value of q
            cur_position = self.fk(q)[:3,3] #Passes in the t-vector from the Jacobian for the end frame
            e = np.subtract(target, cur_position) #Calculates the difference for each index
            return e
        
        #This fn calculates qdot
        def get_qdot(q):
            #Calculate the error
            e = get_error(q)
            #Use error to calculate twist
            twist = K @ e
            #Calculate the Jacobian
            J = self.jacob(q)[:3]
            
            #If using the Jacobian Transpose method
            if method == 'J_T':
                #Calculate the derivative of q
                qdot = J.T @ twist

            #If using the damped psuedo-inverse method
            elif method == 'pinv':
                J_dag = J.T @ np.linalg.inv(J @ J.T + kd**2 * np.eye(3))
                #Calculate the derivative of q
                qdot = J_dag @ twist

            #If the current value of 'method' doesn't match either of the cases above
            else:
                print("Incorrect parameter passed - qdot not calculated")
                qdot = np.zeros((1,len(q)))
            
            return qdot

        #This fn prints the current error value
        def print_error(e, i):
            if (i % 25 == 0):
                print("Current error is:")
                print(e)
                print()
        
        error = get_error(q)
        count = 0
        #This is the while loop where everything happens
        while np.linalg.norm(error) > tol and count < max_iter:
        
            # In this while loop you will update q for each iteration, and update, then
            # your error to see if the problem has converged. You may want to print the error
            # or the "count" at each iteration to help you see the progress as you debug. 
            # You may even want to plot an arm initially for each iteration to make sure 
            # it's moving in the right direction towards the target.     

            qdot = get_qdot(q)
            q = q + qdot
            error = get_error(q)
            #print_error(error, count) #Debugging tool to monitor error magnitudes
            count = count+1
            # input()
        
        # when "while" loop is done, return the relevant info. 

        return (q, error, count, count < max_iter, 'No errors noted')

    def Z_shift(self, R=np.eye(3), p=np.zeros(3,), p_frame='i'):

        """
        Z = Z_shift(R, p, p_frame_order)
        Description: 
            Generates a shifting operator (rotates and translates) to move twists and Jacobians 
            from one point to a new point defined by the relative transform R and the translation p. 

        Parameters:
            R - 3x3 numpy array, expresses frame "i" in frame "j" (e.g. R^j_i)
            p - 3x1 numpy array length 3 iterable, the translation from the initial Jacobian point to the final point, expressed in the frame as described by the next variable.
            p_frame - is either 'i', or 'j'. Allows us to define if "p" is expressed in frame "i" or "j", and where the skew symmetrics matrix should show up. 

        Returns:
            Z - 6x6 numpy array, can be used to shift a Jacobian, or a twist
        """

        # generate our skew matrix
        S = skew(p)

        # generate the matrix that holds the rotation matrices
        ZwR = np.zeros((6,6))
        ZwR[:3,:3] = ZwR[3:,3:] = R
        
        # generate the matrix that hold the identity & skew matrices
        ZwI = np.eye(6)
        ZwI[:3,3:] = -S # don't forget the negative sign!
        
        if p_frame == 'i':
            Z = ZwR @ ZwI
        elif p_frame == 'j':
            Z = ZwI @ ZwR
        else:
            Z = None

        return Z

if __name__ == "__main__":
    from visualization import VizScene
    import time

    # Defining a table of DH parameters where each row corresponds to another joint.
    # The order of the DH parameters is [theta, d, a, alpha] - which is the order of operations. 
    # The symbolic joint variables "q" do not have to be explicitly defined here. 
    # This is a two link, planar robot arm with two revolute joints. 
    dh = [[0, 0, 0.3, 0],
          [0, 0, 0.3, 0]]

    # make robot arm (assuming all joints are revolute)
    arm = SerialArm(dh)

    # defining joint configuration
    q = [pi/4.0, pi/4.0]  # 45 degrees and 45 degrees

    # show an example of calculating the entire forward kinematics
    Tn_in_0 = arm.fk(q)
    print("Tn_in_0:\n", Tn_in_0, "\n")

    # show an example of calculating the kinematics between frames 0 and 1
    T1_in_0 = arm.fk(q, index=[0,1])
    print("T1_in 0:\n", T1_in_0, "\n")

    # showing how to use "print" with the arm object
    print(arm)

    # now visualizing the coordinate frames that we've calculated
    viz = VizScene()

    viz.add_frame(arm.base, label='base')
    viz.add_frame(Tn_in_0, label="Tn_in_0")
    viz.add_frame(T1_in_0, label="T1_in_0")

    time_to_run = 30
    refresh_rate = 60

    for i in range(refresh_rate * time_to_run):
        viz.update()
        time.sleep(1.0/refresh_rate)
    
    viz.close_viz()
    