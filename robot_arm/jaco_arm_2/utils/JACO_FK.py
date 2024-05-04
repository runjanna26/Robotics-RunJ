"""Forward Kinematics for Jaco2 robotics manipulator
---------------------------------------------------------
This file calculates the transformation matrix and the
position of the end-effector w.r.t the base of the robot
Input:
        qjaco: a numpy array including 6 joints angles
Output:
        T: transformation matrix between the end-effector and the base
        E: position of the end-effector
Example:
        qjaco1 = np.array([[261.74, 171.01, 67.6, 152.93, 37.12, 6.91]])
        jacoFK(qjaco1)
---------------------------------------------------------
        Code: Reza Ahmadzadeh, Roshni Kaushik (IRIM-2016)
        email: reza.ahmadzadeh@gatech.edu
---------------------------------------------------------
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_robot_arm(transforms):
    # Extract the joint positions from the transformation matrices
    joint_positions = [T[:3, 3] for T in transforms]
    
    # Extract the end effector position from the last transformation matrix
    end_effector_pos = transforms[-1][:3, 3]
    
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the base
    base_pos = [0,0,0]
    ax.scatter(base_pos[0], base_pos[1], base_pos[2], c='b', marker='o')
    
    # Plot the links
    ax.plot([base_pos[0], joint_positions[0][0]],
            [base_pos[1], joint_positions[0][1]],
            [base_pos[2], joint_positions[0][2]], 'b')
    # Plot the links
    for i in range(len(joint_positions)-1):
        ax.plot([joint_positions[i][0], joint_positions[i+1][0]],
                [joint_positions[i][1], joint_positions[i+1][1]],
                [joint_positions[i][2], joint_positions[i+1][2]], 'b')
    
    # Plot the joints
    for joint_pos in joint_positions:
        ax.scatter(joint_pos[0], joint_pos[1], joint_pos[2], c='r', marker='o')
    
    # Plot the end effector
    ax.scatter(end_effector_pos[0], end_effector_pos[1], end_effector_pos[2], c='k', marker='o')
    
    # Set plot limits and labels
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Display the plot
    plt.show()


transformation_matrices = []
def jacoFK(qjaco):
    global transformation_matrices
    s = qjaco.shape
    if s[0] != 1 or s[1] != 6:
        print('The input vector must be size 1x6')
        return
    q = qjaco + np.array([[0, -90, 90, 0, -180, 90]])
    q[0, 0] *= -1
    q = q*np.pi/180.0
    T,transformation_matrices = forward_kinematics(q)
    E = T[0:3, 3]
    print('T:')
    print(transformation_matrices)
    print('End Effector:')
    print(E)


def forward_kinematics(q):
    num_joints = q.shape[1]
    transformation_matrices = []
    T = np.eye(4)
    for ii in range(num_joints):
        A = calculateA(q[0, ii], ii)
        T = np.dot(T, A)
        transformation_matrices.append(T)
    return T,transformation_matrices


def calculateA(q, n):
    D1 = 0.2755   # base to elbow
    D2 = 0.4100   # arm length
    D3 = 0.2073   # front arm length
    D4 = 0.0741   # first wrist length
    D5 = 0.0741   # second wrist length
    D6 = 0.1600   # wrist to center of the hand
    e2 = 0.0098   # joint 3-4 lateral offset
    aa = 30*np.pi/180
    ca = np.cos(aa)
    sa = np.sin(aa)
    c2a = np.cos(2*aa)
    s2a = np.sin(2*aa)
    d4b = D3 + sa/s2a*D4
    d5b = (sa/s2a*D4 + sa/s2a*D5)
    d6b = (sa/s2a*D5 + D6)

    alpha = np.array([[np.pi/2, np.pi, np.pi/2, 2*aa, 2*aa, np.pi]])
    a = np.array([[0.0, D2, 0.0, 0.0, 0.0, 0.0]])
    d = np.array([[D1, 0.0, -e2, -d4b, -d5b, -d6b]])
                
    A = np.array([[np.cos(q), -np.sin(q)*np.cos(alpha[0, n]), np.sin(q)*np.sin(alpha[0, n]), a[0, n] * np.cos(q)],
                  [np.sin(q), np.cos(q)*np.cos(alpha[0, n]), -np.cos(q)*np.sin(alpha[0, n]), a[0, n] * np.sin(q)],
                  [0.0, np.sin(alpha[0, n]), np.cos(alpha[0, n]), d[0, n]], [0.0, 0.0, 0.0, 1.0]])
    return A
    
# qjaco1 = np.array([[261.74, 171.01, 67.6, 152.93, 37.12, 6.91]])
qjaco1 = np.array([[270, 180, 180, 0, 0, 0]])
jacoFK(qjaco1)
plot_robot_arm(transformation_matrices)