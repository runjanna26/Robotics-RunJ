#!/usr/bin/env python3
'''
v4l2-ctl --list-formats-ext -d /dev/video10
[Command for running this node]
$ ros2 run hero_delta_station_pkg HERO_DELTA_Station --ros-args -p camera_used:=False -p robot_node_used:=False <<< wheel only
$ ros2 run hero_delta_station_pkg HERO_DELTA_Station --ros-args -p camera_used:=False -p robot_node_used:=True  <<< wheel + leg 
$ ros2 run hero_delta_station_pkg HERO_DELTA_Station --ros-args -p camera_used:=True -p robot_node_used:=False  <<< wheel + camera
$ ros2 run hero_delta_station_pkg HERO_DELTA_Station --ros-args -p camera_used:=True -p robot_node_used:=True   <<< leg + wheel + camera
'''
import paramiko
from scp import SCPClient



import rclpy
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray, Float32MultiArray, UInt16MultiArray, Int16MultiArray, Int8, String, UInt16, Int8MultiArray, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy,Imu, Image
from std_srvs.srv import SetBool, Trigger
from tutorial_interfaces.srv import GetInitialPosition


import numpy as np
import time
import os
import cv2
from cv_bridge import CvBridge
from tqdm import tqdm



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('hero_delta_station_node')
        self.declare_parameter('camera_used', 0)
        self.declare_parameter('robot_node_used', 0)
        camera_used        = self.get_parameter('camera_used').get_parameter_value().bool_value
        robot_node_used    = self.get_parameter('robot_node_used').get_parameter_value().bool_value

        # self.version = 'sim'
        self.version = 'real'
        self.camera_used        = bool(camera_used)
        self.robot_node_used    = bool(robot_node_used)

        # ================== Publisher Init ==================
        # Leg module
        self.pub_rob_mode   = self.create_publisher(String,             '/Sanpoom/RobotModes', 1)
        self.pub_qd         = self.create_publisher(Float32MultiArray,  '/Sanpoom/LegModule/JointCommands', 1)
        self.pub_qdd        = self.create_publisher(Float32MultiArray,  '/Sanpoom/LegModule/JointVelocityCommands', 1)
        self.pub_comp       = self.create_publisher(Float32MultiArray,  '/Sanpoom/LegModule/JointCompliance', 1)
        # Wheel modules
        self.pub_wf         = self.create_publisher(Float32MultiArray,  '/Sanpoom/WheelModule/Front/Command', 1)
        self.pub_wb         = self.create_publisher(Float32MultiArray,  '/Sanpoom/WheelModule/Back/Command', 1)
        self.pub_bc_wf      = self.create_publisher(Int8MultiArray,     '/Sanpoom/WheelModule/Front/BoardCommand', 1)
        self.pub_bc_wb      = self.create_publisher(Int8MultiArray,     '/Sanpoom/WheelModule/Back/BoardCommand', 1)
        self.pub_en_wf      = self.create_publisher(Int8MultiArray,     '/Sanpoom/WheelModule/Front/MotorEnable', 1)
        self.pub_en_wb      = self.create_publisher(Int8MultiArray,     '/Sanpoom/WheelModule/Back/MotorEnable', 1)
        self.pub_led_wf     = self.create_publisher(Int8,               '/Sanpoom/WheelModule/Front/LED', 1)           
        self.pub_led_wb     = self.create_publisher(Int8,               '/Sanpoom/WheelModule/Back/LED', 1)            
        self.pub_con_wf     = self.create_publisher(Bool,               '/Sanpoom/WheelModule/Front/ConnectionStationWheel', 1)
        self.pub_con_wb     = self.create_publisher(Bool,               '/Sanpoom/WheelModule/Back/ConnectionStationWheel', 1)

        self.pub_tran = self.create_publisher(Float32MultiArray,        '/Sanpoom/NeuralTransition/States', 1)
        
        # Inspection Tool
        self.pub_yoke           = self.create_publisher(Float32MultiArray,  '/Sanpoom/InspectionTool/YokeCommand', 1)
        self.pub_uvled          = self.create_publisher(Int8,               '/Sanpoom/InspectionTool/UVLED', 1)
        self.pub_wled           = self.create_publisher(Int8,               '/Sanpoom/InspectionTool/LED', 1)          
        self.pub_liquidpump     = self.create_publisher(Int8,               '/Sanpoom/InspectionTool/LiquidPump', 1)
        self.high_res_state_pub = self.create_publisher(Bool,               '/Sanpoom/InspectionCamera/HighResolutionImages/State', 10)
        # Debug
        self.pub_debug          = self.create_publisher(Float32MultiArray, '/Debug', 1)


        print('[STATUS] Init ROS publisher and subscriber')




        self.prev_button_state = False
        self.button_state = False
        
        self.updated_wheel_reset_cmd = False
        self.updated_wheel_reset_cmd_ros = False
        

        self.trans_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        self.wf_msg = Float32MultiArray()
        self.wb_msg = Float32MultiArray()

        self.Xee_Front = [0.0,0.0,0.0]
        self.Xee_Back = [0.0,0.0,0.0]
        self.Xee_d_Front = [0.0,0.0,0.0]
        self.Xee_d_Back = [0.0,0.0,0.0]

        self.axes_js = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.btn_js =  (0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

        self.IMU_f = {'ang_vel_x':0.0, 'ang_vel_y':0.0, 'ang_vel_z':0.0, 'roll':0.0, 'pitch':0.0, 'yaw':0.0}
        self.IMU_c = {'ang_vel_x':0.0, 'ang_vel_y':0.0, 'ang_vel_z':0.0, 'roll':0.0, 'pitch':0.0, 'yaw':0.0}
        self.IMU_b = {'ang_vel_x':0.0, 'ang_vel_y':0.0, 'ang_vel_z':0.0, 'roll':0.0, 'pitch':0.0, 'yaw':0.0}

        self.acc_c = {'x':0.0, 'y':0.0, 'z':0.0}

        self.trans_state = []

        

        self.q_cmd = None
        self.qdd_cmd = None
        self.jc_cmd = None
        self.pump_cmd:bool = False                  # ON/OFF
        self.uv_cmd:bool = False                    # ON/OFF
        self.wf_cmd = [0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0]
        self.wb_cmd = [0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0]
        self.cam_cmd = None
        self.wled_cmd = {}
        self.wled_cmd_prev = {}
        
        self.wf_en_cmd          = np.ones(8)
        self.wf_en_cmd_prev     = np.ones(8)
        self.wb_en_cmd          = np.ones(8)
        self.wb_en_cmd_prev     = np.ones(8)
        self.wf_rst_cmd         = np.ones(1)
        self.wb_rst_cmd         = np.ones(1)
        self.con_wf_state       = False
        self.con_wb_state       = False

        print('================[STATUS] Init parameters================')

        ros_node_freq = 100 #Hz
        self.timer = self.create_timer((1/ros_node_freq), self.timer_callback)
        self.start_time = time.time()



    # Main Loop
    def timer_callback(self):   
        # ========= [ROS Publish control command] =========

        # Parameters
        amplitude = np.deg2rad(60)  # Amplitude of the sine wave
        frequency = 1  # Frequency in Hz
        phase = 0  # Phase shift


        current_time = time.time() - self.start_time  # Elapsed time
        y = amplitude * np.sin(2 * np.pi * frequency * current_time + phase)

        self.wb_cmd[1] = np.deg2rad(90)


        if y < 0.0:
            self.wled_cmd['back'] = 0
        else:
            self.wled_cmd['back'] = 1




        # Station to Wheel module Connection command publish
        con_msg = Bool()
        con_msg.data = True
        self.pub_con_wf.publish(con_msg)
        self.pub_con_wb.publish(con_msg)

        # Wheel modules command publish
        self.wf_msg.data = [float(x) for x in self.wf_cmd]
        self.wb_msg.data = [float(x) for x in self.wb_cmd]
        self.pub_wf.publish(self.wf_msg)
        self.pub_wb.publish(self.wb_msg)



        if not np.array_equal(self.wled_cmd , self.wled_cmd_prev):
        #     # White LEDs command Publish
            wled_msg = Int8()
        #     # White LED front wheel module
        #     wled_msg.data = self.wled_cmd['front']
        #     self.pub_led_wf.publish(wled_msg)
        #     # White LED inspection tool
        #     wled_msg.data = self.wled_cmd['tool']
        #     self.pub_wled.publish(wled_msg)
            # White LED back wheel module
            wled_msg.data = self.wled_cmd['back']
            self.pub_led_wb.publish(wled_msg)

            self.wled_cmd_prev = self.wled_cmd.copy()


        # Magnetic particle pump command publish
        pump_msg = Int8()
        pump_msg.data = self.pump_cmd
        self.pub_liquidpump.publish(pump_msg)

        
        # Reset state connection
        self.con_wf_state       = False
        self.con_wb_state       = False
            




def main(args=None):

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("=============HERO Station [Terminated]=============")

    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


