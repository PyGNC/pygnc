#MEKF helper functions
import autograd.numpy as np

def hat(omega):

    return np.array([[0, -omega[2], omega[1]],[omega[2], 0, -omega[0]],[-omega[1], omega[0], 0]])

#Left muliply quaternion matrix    
def L(q): 
    #scalar part of quat
    qs = q[0]

    #vector part of quat (dimension 1x3)
    qv = np.expand_dims(q[1:], axis=0)
    
    L = np.block([
    [q[0], -qv], 
    [qv.T, q[0]*np.identity(3)+hat(q[1:])]            
    ])
    
    return L

#Right multiply quaternion matrix
#R_ to not overload with the measurement noise R
def R(q): 

    #scalar part of quat
    qs = q[0]

    #vector part of quat (dimension 1x3)
    qv = np.expand_dims(q[1:], axis=0)
    
    R = np.block([
    [q[0], -qv], 
    [qv.T, q[0]*np.identity(3)-hat(q[1:])]            
    ])
    
    return R

#operator that converts a 3 parameter vector to a quaterion w zero real part
H = np.vstack((np.zeros((1,3)), np.identity(3)))

#Conjugate of a quaternion. Negate the vector part

def conj_q(q):

    invq = np.hstack((q[0], -1*q[1:]))

    return invq

#Quaternion to  Rodrigues Parameter
def quaternion_to_RP(q):

    return q[1:4]/q[0]

#Quaternion to Rotation Matrix
def quaternion_to_rotmatrix(q):

    return H.T@L(q)@R(q).T@H

#Rodrigues Paramter to Quaternion (Cayley Map)
def RP_to_quaternion(rp):

    return  1/np.sqrt(1+np.linalg.norm(rp)**2)*np.hstack((1,rp))