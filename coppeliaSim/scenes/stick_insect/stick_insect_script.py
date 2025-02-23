import numpy as np
import pandas as pd


def sysCall_init():
    sim = require('sim')

    global stick_insect_joint, stick_insect_foot_force
    global leg_0_df, leg_1_df, leg_2_df
    leg_0_df = pd.read_csv('../../Robotics-RunJ/coppeliaSim/scenes/stick_insect/include/stick_insect_path/rbfn_joint_targets_0.csv')
    leg_1_df = pd.read_csv('../../Robotics-RunJ/coppeliaSim/scenes/stick_insect/include/stick_insect_path/rbfn_joint_targets_1_v1.csv')
    leg_2_df = pd.read_csv('../../Robotics-RunJ/coppeliaSim/scenes/stick_insect/include/stick_insect_path/rbfn_joint_targets_2_new_v1.csv')
    

        

    stick_insect_joint      = {'TR': [],'CR': [],'FR': [],'TL': [],'CL': [],'FL': []}
    stick_insect_foot_force = {'R': [], 'L': []}
    stick_insect_body       = sim.getObject('..')
    for i in range(3):
        stick_insect_joint['TR'].append(sim.getObject('../TR'+str(i)))
        stick_insect_joint['CR'].append(sim.getObject('../CR'+str(i)))
        stick_insect_joint['FR'].append(sim.getObject('../FR'+str(i)))
        stick_insect_joint['TL'].append(sim.getObject('../TL'+str(i)))
        stick_insect_joint['CL'].append(sim.getObject('../CL'+str(i)))
        stick_insect_joint['FL'].append(sim.getObject('../FL'+str(i)))
        stick_insect_foot_force['R'].append(sim.getObject('../R'+str(i)+'_fs'))
        stick_insect_foot_force['L'].append(sim.getObject('../L'+str(i)+'_fs'))
    # print(stick_insect_joint)
    # print(stick_insect_foot_force)
    
    initial_joint_angles = {'TR': [30, 0, -40],
                     'CR': [10, 0,  10],
                     'FR': [-60, -60, -60],
                     'TL': [ 30,   0, -40],
                     'CL': [ 10,   0,  10],
                     'FL': [-60, -60, -60]}
    for group, angles in initial_joint_angles.items():   # Set joint target positions using a loop
        for i, angle in enumerate(angles):
            sim.setJointTargetPosition(stick_insect_joint[group][i], np.deg2rad(angle))

    # sim.setObjectInt32Parameter(stick_insect_body, sim.shapeintparam_static, 0) 
     
i = 0
def sysCall_actuation():
    pass
    global i
    
    if i < len(leg_0_df['TR0']):
        sim.setJointTargetPosition(stick_insect_joint['TR'][0], leg_0_df['TR0'][i])
        sim.setJointTargetPosition(stick_insect_joint['CR'][0], leg_0_df['CR0'][i])
        sim.setJointTargetPosition(stick_insect_joint['FR'][0], leg_0_df['FR0'][i])
        
        sim.setJointTargetPosition(stick_insect_joint['TR'][1], leg_1_df['TR1'][i])
        sim.setJointTargetPosition(stick_insect_joint['CR'][1], leg_1_df['CR1'][i])
        sim.setJointTargetPosition(stick_insect_joint['FR'][1], leg_1_df['FR1'][i])        
        
        sim.setJointTargetPosition(stick_insect_joint['TR'][2], leg_2_df['TR2'][i])
        sim.setJointTargetPosition(stick_insect_joint['CR'][2], leg_2_df['CR2'][i])
        sim.setJointTargetPosition(stick_insect_joint['FR'][2], leg_2_df['FR2'][i]) 
        i += 1
    else:
        i = 0

def sysCall_sensing():
    pass

def sysCall_cleanup():
    pass

#======================================================================================
def rotx(a):
    a = np.radians(a)
    return np.matrix([[1,   0,     0,  0],
                      [0, c(a), -s(a),  0],
                      [0, s(a),  c(a),  0],
                      [0,    0,     0,  1]])
def c(x):
    return np.cos(x)
def s(x):
    return np.sin(x)