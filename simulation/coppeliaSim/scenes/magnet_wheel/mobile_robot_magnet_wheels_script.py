import numpy as np


def sysCall_init():
    sim = require('sim')

    global force
    global res, dist, point, obj, n
    res     = [0,0,0,0] 
    dist    = [0,0,0,0]
    point   = [0,0,0,0]
    obj     = [0,0,0,0]
    n       = [0,0,0,0]
    global Wheel 
    global Magnet
    
    
    Wheel = [0,0,0,0]
    Magnet = [0,0,0,0]
    for i in range(len(Wheel)):
        Wheel[i]  = sim.getObject('../wheel_motor_'+ str(i) + '/magnet_wheel')
        Magnet[i] = sim.getObject('../wheel_motor_'+ str(i) + '/magnet_wheel/magnet_wheel_range')

    force = 30 # magnetic force in Newton [N] 

def sysCall_actuation():
    pass

def sysCall_sensing():
    for i in range(4):
        res[i], dist[i], point[i], obj[i], n[i]  = sim.readProximitySensor(Magnet[i])   # detected point of proximity sensor ("magnet_wheel_range")
        if res[i]==1:
            # unit vector of detected points
            point[i][0] = point[i][0]/dist[i]
            point[i][1] = point[i][1]/dist[i]
            point[i][2] = point[i][2]/dist[i]
        
            point_mat = np.matrix( [[1,   0,    0,    point[i][0]],
                                    [0,   1,    0,    point[i][1]],
                                    [0,   0,    1,    point[i][2]],
                                    [0,   0,    0,             1]])          
            point_wheel_frame = rotx(90.0) @ point_mat  # convert detected point from "magnet_wheel_range" frame to "magnet_wheel" frame
            sim.addForce(Wheel[i],[0,0,0],[force*point_wheel_frame[0,3],force*point_wheel_frame[1,3],force*point_wheel_frame[2,3]]) # apply force vector to "magnet_wheel" in the direction of detected point 
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