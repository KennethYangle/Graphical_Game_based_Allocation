#!/usr/bin/env python
# -*- coding: utf-8 -*-

import io
import os
import time
import numpy as np

import json5




from utils import constrain_rad

file_pwd = os.path.dirname(os.path.abspath(__file__))
time_prefix = time.strftime("%Y-%m-%d_%H-%M-%S")




class TaskBase:
    def __init__(self,px4_control):
        self.px4_control = px4_control


    def task_test_rpy(self):
        px4_control = self.px4_control
        while True:
            yaw_offset = np.rad2deg(px4_control.mav_yaw_offset)
            yaw = np.rad2deg(px4_control.mav_yaw)
            roll = np.rad2deg(px4_control.mav_roll)
            pitch = np.rad2deg(px4_control.mav_pitch) 
            print("rpy: {:.2f} {:.2f} {:.2f} {:.2f}".format(roll,pitch,yaw,yaw_offset))
            time.sleep(1)


    def task_test_pose(self):
        px4_control = self.px4_control
        while True:
            yaw = np.rad2deg(px4_control.mav_yaw)
            yaw_offset = np.rad2deg(px4_control.mav_yaw_offset)
            yaw_odom = np.rad2deg(px4_control.mav_yaw_odom)
            print("yaw: {:.2f} {:.2f} {:.2f}".format(yaw,yaw_offset,yaw_odom))
            print("pos_enu: {}".format(px4_control.pos_enu))
            print("pos_odom: {}".format(px4_control.pos_odom))
            print("pos_swarm: {}".format(px4_control.pos_swarm))
            print()
            time.sleep(1)


    def task_test_vel(self):
        '''
        速度指令FLU
        '''
        px4_control = self.px4_control
        
        px4_control.moveByVelocityYawrateBodyFrame(vx=0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateBodyFrame(vy=0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateBodyFrame(yaw_rate=np.deg2rad(45))
        time.sleep(2)

        px4_control.moveByVelocityYawrateBodyFrame(vx=-0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateBodyFrame(vy=0.5)
        time.sleep(5)
    
    def task_test_velENU(self):
        '''
        速度指令ENU
        '''
        px4_control = self.px4_control
        
        px4_control.moveByVelocityYawrateENU(vx=0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateENU(vy=0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateENU(yaw_rate=np.deg2rad(45))
        time.sleep(2)

        px4_control.moveByVelocityYawrateENU(vx=-0.5)
        time.sleep(5)
        px4_control.moveByVelocityYawrateENU(vy=-0.5)
        time.sleep(5)
    
    def task_test_vel_fast(self):
        '''
        速度指令FLU
        '''
        px4_control = self.px4_control
        
        px4_control.moveByVelocityYawrateBodyFrame(vx=8)
        time.sleep(8)
        px4_control.moveByVelocityYawrateBodyFrame()
        time.sleep(5)

    
    def task_test_pos(self):
        '''
        位置指令ENU
        '''
        px4_control = self.px4_control
        
        px4_control.moveByPosENU(x=2)
        time.sleep(5)
        px4_control.moveByPosENU(y=2)
        time.sleep(5)
        px4_control.moveByPosENU(z=3)
        time.sleep(5)

    
    def task_test_swarm_pos(self):
        px4_control = self.px4_control
        
        px4_control.moveBySwarmPosEN(x=0,y=2)
        time.sleep(5)
        px4_control.moveBySwarmPosEN(x=2,y=2)
        time.sleep(5)
        px4_control.moveBySwarmPosEN(x=3,y=3)
        time.sleep(5)