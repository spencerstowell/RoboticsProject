"""
Transforms Module - Contains code for to learn about rotations
and eventually homogenous transforms. 

Empty outline derived from code written by John Morrell. 
"""

import numpy as np
from numpy.linalg import norm

## 2D Rotations
def rot2(th):
    """
    R = rot2(theta)
    Parameters
        theta: float or int, angle of rotation
    Returns
        R: 2 x 2 numpy array representing rotation in 2D by theta
    """
    R = np.array([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]])
    return clean_rotation_matrix(R)

## 3D Transformations
def rotx(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """
    R = np.array([[1, 0, 0],[0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]])

    return clean_rotation_matrix(R)

def roty(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """
    R = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0], [-np.sin(th), 0, np.cos(th)]])

    return clean_rotation_matrix(R)

def rotz(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """
    R = np.array([[np.cos(th), -np.sin(th), 0],
                  [np.sin(th), np.cos(th), 0],
                  [0, 0, 1]])

    return clean_rotation_matrix(R)

# inverse of rotation matrix 
def rot_inv(R):
    '''
    R = rot_inv(R)
    Parameters
        R: 2x2 or 3x3 numpy array representing a proper rotation matrix
    Returns
        R: 2x2 or 3x3 inverse of the input rotation matrix
    '''
    R = R.T
    return clean_rotation_matrix(R)

def se3(R=np.eye(3), p=np.array([0, 0, 0])):
    """
        T = se3(R, p)
        Description:
            Given a numpy 3x3 array for R, and a 1x3 or 3x1 array for p, 
            this function constructs a 4x4 homogeneous transformation 
            matrix "T". 

        Parameters:
        R - 3x3 numpy array representing orientation, defaults to identity
        p = 3x1 numpy array representing position, defaults to [0, 0, 0]

        Returns:
        T - 4x4 numpy array
    """
    T = np.array([[R[0, 0], R[0, 1], R[0, 2], p[0]],
                  [R[1, 0], R[1, 1], R[1, 2], p[1]],
                  [R[2, 0], R[2, 1], R[2, 2], p[2]],
                  [0, 0, 0, 1]])

    return clean_rotation_matrix(T)

def inv(T):
    """
        Tinv = inv(T)
        Description:
        Returns the inverse transform to T

        Parameters:
        T

        Returns:
        Tinv - 4x4 numpy array that is the inverse to T so that T @ Tinv = I
    """
    
    R = np.array([[T[0, 0], T[0, 1], T[0, 2]],
                  [T[1, 0], T[1, 1], T[1, 2]],
                  [T[2, 0], T[2, 1], T[2, 2]]])
    #print(R)
    p = np.array([T[0, 3], T[1, 3], T[2, 3]])
    #print(p)
    R_inv = R.T
    p_inv = -R_inv @ p
    T_inv = np.array([[R_inv[0, 0], R_inv[0, 1], R_inv[0, 2], p_inv[0]],
                      [R_inv[1, 0], R_inv[1, 1], R_inv[1, 2], p_inv[1]],
                      [R_inv[2, 0], R_inv[2, 1], R_inv[2, 2], p_inv[2]],
                      [0, 0, 0, 1]])

    return clean_rotation_matrix(T_inv)

def clean_rotation_matrix(R, eps=1e-12):
    '''
    This function is not required, but helps to make sure that all
    matrices we return are proper rotation matrices
    '''

    for i in range(R.shape[0]):
        for j in range(R.shape[1]):
            if np.abs(R[i, j]) < eps:
                R[i, j] = 0.
            elif np.abs(R[i, j] - 1) < eps:
                R[i, j] = 1.
    return R

def R2rpy(R):
    """
    rpy = R2rpy(R)
    Description:
    Returns the roll-pitch-yaw representation of the SO3 rotation matrix

    Parameters:
    R - 3 x 3 Numpy array for any rotation

    Returns:
    rpy - 1 x 3 Numpy Matrix, containing <roll pitch yaw> coordinates (in radians)
    """
    
    # follow formula in book (pg. 52), use functions like "np.atan2" 
    # for the arctangent and "**2" for squared terms. 

    roll = np.arctan2(R[1,0],R[0,0])
    pitch = np.arctan2(-R[2,0],np.sqrt(R[2,1]**2 + R[2,2]**2))
    yaw = np.arctan2(R[2,1],R[2,2])

    return np.array([roll, pitch, yaw])

def R2axis(R):
    """
    axis_angle = R2axis(R)
    Description:
    Returns an axis angle representation of a SO(3) rotation matrix

    Parameters:
    R

    Returns:
    axis_angle - 1 x 4 numpy matrix, containing  the axis angle representation
    in the form: <angle, rx, ry, rz>
    """

    # see equation (2.27) and (2.28) on pg. 54, using functions like "np.acos," "np.sin," etc. 
    ang = np.arccos((R[0,0] + R[1,1] + R[2,2] - 1) / 2) # TODO - fill out here. 
    axis_angle = np.array([ang,
                           (R[2,1] - R[1,2]) / (2*np.sin(ang)), # TODO - fill out here, each row will be a function of "ang"
                           (R[0,2] - R[2,0]) / (2*np.sin(ang)),
                           (R[1,0] - R[0,1]) / (2*np.sin(ang))])

    return axis_angle

def axis2R(ang, v):
    """
    R = axis2R(angle, rx, ry, rz, radians=True)
    Description:
    Returns an SO3 object of the rotation specified by the axis-angle

    Parameters:
    angle - float, the angle to rotate about the axis in radians
    v = [rx, ry, rz] - components of the unit axis about which to rotate as 3x1 numpy array
    
    Returns:
    R - 3x3 numpy array
    """
    # TODO fill this out
    R = np.array([[v[0]**2*(1-np.cos(ang))+np.cos(ang), v[0]*v[1]*(1-np.cos(ang))-v[2]*np.sin(ang), v[0]*v[2]*(1-np.cos(ang))+v[1]*np.sin(ang)],
                 [v[0]*v[1]*(1-np.cos(ang))+v[2]*np.sin(ang), v[1]**2*(1-np.cos(ang))+np.cos(ang), v[1]*v[2]*(1-np.cos(ang))-v[0]*np.sin(ang)],
                 [v[0]*v[2]*(1-np.cos(ang))-v[1]*np.sin(ang), v[1]*v[2]*(1-np.cos(ang))+v[0]*np.sin(ang), v[2]**2*(1-np.cos(ang))+np.cos(ang)]])
    return clean_rotation_matrix(R)

def R2quat(R):
    """
    quaternion = R2quat(R)
    Description:
    Returns a quaternion representation of pose

    Parameters:
    R

    Returns:
    quaternion - 1 x 4 numpy matrix, quaternion representation of pose in the 
    format [nu, ex, ey, ez]
    """
    # see equation (2.34) and (2.35) on pg. 55, using functions like "sp.sqrt," and "sp.sign"

    return np.array([np.sqrt(R[0,0] + R[1,1] + R[2,2] + 1) / 2,
                     np.sign(R[2,1] - R[1,2])*np.sqrt(R[0,0] - R[1,1] - R[2,2] + 1) / 2,
                     np.sign(R[0,2] - R[2,0])*np.sqrt(R[1,1] - R[2,2] - R[0,0] + 1) / 2,
                     np.sign(R[1,0] - R[0,1])*np.sqrt(R[2,2] - R[0,0] - R[1,1] + 1) / 2])
             
def quat2R(q):
    """
    R = quat2R(q)
    Description:
    Returns a 3x3 rotation matrix

    Parameters:
    q - 4x1 numpy array, [nu, ex, ey, ez ] - defining the quaternion
    
    Returns:
    R - a 3x3 numpy array 
    """
    # TODO, extract the entries of q below, and then calculate R
    nu = q[0]
    ex = q[1]
    ey = q[2]
    ez = q[3]
    R =  np.array([[2*(nu**2+ex**2)-1, 2*(ex*ey-nu*ez), 2*(ex*ez+nu*ey)],
                   [2*(ex*ey+nu*ez), 2*(nu**2+ey**2)-1, 2*(ey*ez-nu*ex)],
                   [2*(ex*ez-nu*ey), 2*(ey*ez+nu*ex), 2*(nu**2+ez**2)-1]])
    return clean_rotation_matrix(R)

def euler2R(th1, th2, th3, order='xyz'):
    """
    R = euler2R(th1, th2, th3, order='xyz')
    Description:
    Returns a 3x3 rotation matrix as specified by the euler angles, we assume in all cases
    that these are defined about the "current axis," which is why there are only 12 versions 
    (instead of the 24 possiblities noted in the course slides). 

    Parameters:
    th1, th2, th3 - float, angles of rotation
    order - string, specifies the euler rotation to use, for example 'xyx', 'zyz', etc.
    
    Returns:
    R - 3x3 numpy matrix
    """

    # TODO - fill out each expression for R based on the condition 
    # (hint: use your rotx, roty, rotz functions)
    if order == 'xyx':
        R = rotx(th1) @ roty(th2) @ rotx(th3)
    elif order == 'xyz':
        R = rotx(th1) @ roty(th2) @ rotz(th3)
    elif order == 'xzx':
        R = rotx(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'xzy':
        R = rotx(th1) @ rotz(th2) @ roty(th3)
    elif order == 'yxy':
        R = roty(th1) @ rotx(th2) @ roty(th3)
    elif order == 'yxz':
        R = roty(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'yzx':
        R = roty(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'yzy':
        R = roty(th1) @ rotz(th2) @ roty(th3)
    elif order == 'zxy':
        R = rotz(th1) @ rotx(th2) @ roty(th3)
    elif order == 'zxz':
        R = rotz(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'zyx':
        R = rotz(th1) @ roty(th2) @ rotx(th3)
    elif order == 'zyz':
        R = rotz(th1) @ roty(th2) @ rotz(th3)
    else:
        print("Invalid Order!")
        return

    return clean_rotation_matrix(R)