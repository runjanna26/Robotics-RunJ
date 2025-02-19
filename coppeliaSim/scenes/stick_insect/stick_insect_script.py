import numpy as np

def sysCall_init():
    sim = require('sim')

    global stick_insect_joint, stick_insect_foot_force

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
    
    target_angles = {'TR': [30, 0, -40],
                     'CR': [10, 0,  10],
                     'FR': [-60, -60, -60],
                     'TL': [ 30,   0, -40],
                     'CL': [ 10,   0,  10],
                     'FL': [-60, -60, -60]}
    for group, angles in target_angles.items():   # Set joint target positions using a loop
        for i, angle in enumerate(angles):
            sim.setJointTargetPosition(stick_insect_joint[group][i], np.deg2rad(angle))

    sim.setObjectInt32Parameter(stick_insect_body, sim.shapeintparam_static, 0)  

def sysCall_actuation():
    pass

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