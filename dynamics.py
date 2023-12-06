"""
dynamics Module - Contains code for:
- Dynamic SerialArm class
- RNE Algorithm
- Euler - Lagrange formulation

John Morrell, Jan 28 2022
Tarnarmour@gmail.com

modified by: 
Marc Killpack, Nov. 4, 2022
"""

import numpy as np
from kinematics import SerialArm
from utility import skew

eye = np.eye(4)

class SerialArmDyn(SerialArm):
    """
    SerialArmDyn class represents serial arms with dynamic properties and is used to calculate forces, torques, accelerations,
    joint forces, etc. using the Newton-Euler and Euler-Lagrange formulations. It inherits from the previously defined kinematic
    robot arm class "SerialArm". 
    """

    def __init__(self, 
                 dh, 
                 jt=None, 
                 base=eye, 
                 tip=eye, 
                 joint_limits=None,
                 mass=None,
                 r_com=None,
                 link_inertia=None,
                 motor_inertia=None,
                 joint_damping=None):

        SerialArm.__init__(self, dh, jt, base, tip, joint_limits)
        self.mass = mass
        self.r_com = r_com
        self.link_inertia = link_inertia
        self.motor_inertia = motor_inertia
        if joint_damping is None:
            self.B = np.zeros((self.n, self.n))
        else:
            self.B = np.diag(joint_damping)

    def rne(self, q, qd, qdd, 
            Wext=np.zeros((6,)),
            g=np.zeros((3,)),
            omega_base=np.zeros((3,)),
            alpha_base=np.zeros((3,)),
            v_base=np.zeros((3,)),
            acc_base=np.zeros((3,))):

        """
        tau, W = RNE(q, qd, qdd):
        returns the torque in each joint (and the full wrench at each joint) given the joint configuration, velocity, and accelerations
        Args:
            q:
            qd:
            qdd:

        Returns:
            tau: torques or forces at joints (assuming revolute joints for now though)
            wrenches: force and torque at each joint, and for joint i, the wrench is in frame i


        We start with the velocity and acceleration of the base frame, v0 and a0, and the joint positions, joint velocities,
        and joint accelerations (q, qd, qdd).

        For each joint, we find the new angular velocity, w_i = w_(i-1) + z * qdot_(i-1)
        v_i = v_(i-1) + w_i x r_(i-1, com_i)


        if motor inertia is None, we don't consider it. Solve for now without motor inertia. The solution will provide code for motor inertia as well. 
        """

        omegas = []
        alphas = []
        acc_ends = []
        acc_coms = []


        # First we'll define some additional terms that we'll use in each iteration of the algorithm
        Rs = []  # List of Ri-1_i, rotation from i-1 to i in the i-1 frame
        R0s = []  # List of R0_i, rotation from 0 to i in the 0 frame
        rp2cs = []  # List of pi-1_i-1_i, vector from i-1 to i frame in frame i-1
        rp2coms = []  # List of r_i_i-1,com, the vector from the origin of frame i-1 to the COM of link i in the i frame
        zaxes = []  # List of z axis of frame i-1, expressed in frame i

        ## Solve for needed rotation and translation vectors
        for i in range(0, self.n):
            # Grab the homogeneous transformation matrix of i-1 in i
            T = self.fk(q, [i, i+1])  # Find the transform from link i to link i+1

            # From that transformation matrix extract the rotation matrix (rows & columns 0-2)
            R = T[0:3, 0:3]

            # From the homogeneous transformation matrix, extract the translation vector from i-1 to i in frame i-1
            p = T[0:3, 3]

            # Store the rotation matrix of i-1 to i in frame i for later use
            Rs.append(R)
            # Store the vector, except now it will be from frame i-1 to i in frame i (rotated)
            rp2cs.append(R.T @ p)
            # Use the vector of center-of-mass moment arms, defined in frame i, with the translation vector (in frame i)
            # to get the vector from i-1 to COM,i
            rp2coms.append(R.T @ p + self.r_com[i])
            # From the rotation matrix, extract the orientation of Z for frame i-1 in frame i (last column of R)
            # Store Z for later
            zaxes.append(Rs[i-1].T[0:3, 2])

            # Calculate the rotation matrix from the base frame to the current one
            # i+1 is used since we're going to a frame, numbered from 1, but our indices started from 0
            R0 = self.fk(q, i+1)[0:3, 0:3]
            R0s.append(R0)
        
        ## Now that we've solved for the necessary parts, initialize some variables to use in
        # the proceeding loop
        w_prev = omega_base
        alph_prev = alpha_base
        a_prev = acc_base

        ## Solve for angular velocities, angular accelerations, and linear accelerations
        for i in range(0, self.n):
            
            if self.jt[i] == 'r':
                # Calculate the current link's angular velocity & acceleration
                w_cur = Rs[i].T @ w_prev + zaxes[i]*qd[i]
                alph_cur = Rs[i].T @ alph_prev + zaxes[i]*qdd[i] + np.cross(w_cur, zaxes[i]) * qd[i]
                
                # Calculate the acceleration at the center of mass and the end of the link
                a_com = Rs[i].T @ a_prev + np.cross(alph_cur, rp2coms[i]) + np.cross(w_cur, np.cross(w_cur, rp2coms[i]))
                a_end = Rs[i].T @ a_prev + np.cross(alph_cur, rp2cs[i]) + np.cross(w_cur, np.cross(w_cur, rp2cs[i]))
            else:
                print("Kinematic Equations for linear motion are needed")

            # Store the values of omega, alpha, and the accelerations
            omegas.append(w_cur)
            alphas.append(alph_cur)
            acc_coms.append(a_com)
            acc_ends.append(a_end)

            # Update the placeholder variables for the next round
            w_prev = w_cur
            alph_prev = alph_cur
            a_prev = a_end
        
        ## Now solve Kinetic equations by starting with forces at last link and going backwards
        ## If helpful, you can define a function to call here so that you can debug the output more easily. 
        Wrenches = np.zeros((6,self.n,))    # Variable to store all forces/torques at each joint
        tau = np.zeros((self.n,))   # Variable to store motor torques

        # Initialize variables prior to the final for loop
        f_prev = Wext[0:3]  # The force on the tip (first 3 indices of external wrench)
        M_prev = Wext[3:]   # The torques on the tip (last 3 indices of external wrench)
        R_ip1_in_frame_i = self.tip[0:3,0:3]    # Rotation of the tip in the nth frame (may just be identity)

        for i in range(self.n - 1, -1, -1):  # Index from n-1 to 0
            
            # Set up the orientation of gravity
            Rg = R0s[i].T   # Get the rotation matrix from frame 0 to frame i in frame i
            # Use Rg to orient gravity properly
            g_cur = Rg @ g

            # Sum of forces and mass * acceleration to find forces
            # m*a = f_cur - f_prev + m*g --> f_cur = m * (a - g) + f_prev
            f_cur = R_ip1_in_frame_i @ f_prev + self.mass[i] * (acc_coms[i] - g_cur)
            
            # Using the sum of moments and d/dt(angular momentum) to find moment at joint
            # Be very careful with the r x f terms here; easy to mess up
            M_cur = self.link_inertia[i] @ alphas[i] \
                + np.cross(omegas[i], self.link_inertia[i] @ omegas[i]) \
                + R_ip1_in_frame_i @ M_prev \
                + np.cross(self.r_com[i], - (R_ip1_in_frame_i @ f_prev)) \
                + np.cross(rp2coms[i], f_cur)

            # store the total force and torque at joint i in the Wrench output. 
            Wrenches[0:3,i] = f_cur
            Wrenches[3:,i] = M_cur

            # Store our intermediate values for the next round of calculations
            R_ip1_in_frame_i = Rs[i]
            f_prev = f_cur
            M_prev = M_cur

        # Now take the information in Wrenches (the torques at each joint in their respective frames)
        # and convert it into the motor torques
        for i in range(self.n):
            if self.jt[i] == 'r':
                # this is the same as doing R_(i-1)^i @ tau_i^i and taking only the third element. 
                tau[i] = zaxes[i] @ Wrenches[3:, i] 
            else:
                print("you need to implement generalized force calculation for joint type:\t", self.jt[i])

        return tau, Wrenches

    # This function returns the mass matrix of the arm
    def get_M(self, q):
        # Initialize M to be n x n
        M = np.zeros((self.n,self.n))

        # Iterate through all of the links
        for i in range(self.n):
            # Set all joint accelerations to 0
            qdd = np.zeros((self.n,))
            # Change a single index (corresponding to the link of interest)
            # to 1
            qdd[i] = 1
            # Calculate the column of M that corresponds to that link
            M[:,i], _ = self.rne(q, np.zeros((self.n,)), qdd)

        # print(M)
        return M
    
    def get_C(self, q, qd):
        # Calculate the coriolis matrix multiplied by qdot
        Cq_dot, _ = self.rne(q, qd, np.zeros((self.n,)))

        # print(Cq_dot)
        return Cq_dot
    
    def get_G(self, q, g):
        # Calculate the gravity matrix, only depends on the joints & gravity
        G, _ = self.rne(q, np.zeros((self.n,)), np.zeros((self.n,)), g=g)

        # print(G)
        return G

if __name__ == '__main__':

    ## this just gives an example of how to define a robot, this is a planar 3R robot.
    dh = [[0, 0, 0.4, 0],
        [0, 0, 0.4, 0],
        [0, 0, 0.4, 0]]

    joint_type = ['r', 'r', 'r']

    link_masses = [1, 1, 1]

    # defining three different centers of mass, one for each link
    r_coms = [np.array([-0.2, 0, 0]), np.array([-0.2, 0, 0]), np.array([-0.2, 0, 0])]

    link_inertias = []
    for i in range(len(joint_type)):
        iner = 0.01
        # this inertia tensor is only defined as having Izz non-zero
        link_inertias.append(np.array([[0, 0, 0], [0, 0, 0], [0, 0, iner]]))


    arm = SerialArmDyn(dh,
                       jt=joint_type,
                       mass=link_masses,
                       r_com=r_coms,
                       link_inertia=link_inertias)

    # once implemented, you can call arm.RNE and it should work. 
    q = [np.pi/4.0]*3
    qd = [np.pi/6.0, -np.pi/4.0, np.pi/3.0]
    qdd = [-np.pi/6.0, np.pi/3.0, np.pi/6.0]
    tau, _ = arm.rne(q, qd, qdd, g=np.array([0, -9.81, 0]))

    print(tau)
    arm.get_M(q)
    arm.get_C(q, qd)