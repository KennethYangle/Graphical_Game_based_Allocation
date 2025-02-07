#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np

from utils import saturation,constrain_rad


XYR_CTL = 10
XYR_THROUGH = 11
XYR_LOSS = 12


FRD_CTL = 20
FRD_THROUGH = 21
FRD_COUNT = 22



class visual_servo_bs:
    def __init__(self):

        self.k_yaw = 0.55
        self.cx = 312.11
        self.cy = 240.94
        self.fx = 610.96
        self.fy = 610.08

        self.n_cc = np.array([0,0,1])
        self.R_cb = np.array([
            [0,0,1],
            [1,0,0],
            [0,1,0]
        ])

        # 动力学参数
        self.R_be = np.identity(3)
        self.yaw_rad = 0
        self.vs_start_time = time.time()
        self.reset()

    def reset(self):
        self.reset_config()
        self.reset_state()

    def reset_config(self):
        # 会根据配置文件更新的参数
        self.vx = 1.0
        self.vy = 1.2
        self.vz = 1.2

        # 像素视觉伺服控制起始和终止的圆环半径
        self.xyr_R_min = 20
        self.xyr_R_max = 220


        self.frd_time_scale = 0.95

        # frd控制终止条件
        self.frd_through_dis = 2.5
        self.frd_R_max = 180
        self.frd_R_mmax = 220


        # 穿越的保持时间
        self.xyr_through_time = 0.6
        self.frd_through_time = 1.5

        self.xyr_through_enable = False

        self.frd_min_dis = 0.75
        self.frd_min_enable = False


        
    
    def reset_state(self):
        self.vs_state = XYR_CTL
        self.next_task_flag = False

        self.loss_count = 0
        self.through_count = 0
        self.frd_count = 0
        

        self.last_F = 0
        self.last_R = 0
        self.last_v_cmd = np.zeros(3)
        self.last_yaw_rate_cmd = 0
        
        self.state_time = time.time()
    
    def switch_to(self,r):
        result = False
        if self.xyr_R_min < r < self.xyr_R_max:
            self.vs_start_time = time.time()
            self.state_time = time.time()
            result = True

        return result


    def update_kinematics(self,R_be,yaw_rad):
        self.R_be = R_be
        self.yaw_rad = yaw_rad

    def visual_servo_real_update(self,circle_xyr,circle_FRD,tgt_yaw):
        yaw_rate_cmd = self.k_yaw * constrain_rad(tgt_yaw - self.yaw_rad)
        cx,cy,r = circle_xyr
        circle_F,circle_R,circle_D = circle_FRD

        # 状态机转换，日常进行视觉伺服的控制
        if self.vs_state == XYR_CTL:
            # 深度有效，3~5m切换至FRD速度控制
            if (0.8 < circle_F < 5):
                self.vs_state = FRD_CTL
                self.state_time = time.time()
            # 深度无效时使用像素控制，大于像素半径阈值则穿越
            if self.xyr_through_enable and r > self.xyr_R_max:
                self.vs_state = XYR_THROUGH
                self.state_time = time.time()

        elif self.vs_state == FRD_CTL:
            if (
                # 防止误匹配
                ( 0.8 < circle_F < self.frd_through_dis and r > self.frd_R_max)
                or ( r > self.frd_R_mmax )
                # 丢失或者距离特别近
                or ( self.frd_min_enable and 0.1 < circle_F < 1.75 )
            ):
                if time.time() - self.state_time > 0.15:
                    self.vs_state = FRD_THROUGH
                    self.state_time = time.time()

        elif self.vs_state == XYR_THROUGH:
            # 保持速度直到xyr_through_time
            if time.time() - self.state_time >  self.xyr_through_time:
                self.next_task_flag = True
                # 如果视觉控制的时间太短，认为是有问题的，重置
                if time.time() - self.vs_start_time < 1.25:
                    self.reset_state()
        elif self.vs_state == FRD_THROUGH:
            if time.time() - self.state_time >  self.frd_through_time:
                self.next_task_flag = True
                # 如果视觉控制的时间太短，认为是有问题的，重置
                if time.time() - self.vs_start_time < 1.25:
                    self.reset_state()

        if self.vs_state == XYR_CTL:
            v_cmd = self.xyr_velcmd_update(cx, cy)
        elif self.vs_state == FRD_CTL:
            v_cmd = self.frd_velcmd_update(circle_F,circle_R,circle_D)
        elif self.vs_state == XYR_THROUGH:
            v_cmd = self.last_v_cmd
        elif self.vs_state == FRD_THROUGH:
            v_cmd = self.last_v_cmd

        self.last_R = r
        self.last_v_cmd = v_cmd
        return v_cmd, yaw_rate_cmd


    


    def xyr_velcmd_update(self,cx, cy):
        ex_i,ey_i = cx-self.cx,cy-self.cy

        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = self.R_be.dot(n_bc)
        
        #calacute the no
        n_co = np.array([ex_i, ey_i, self.fx], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = self.R_be.dot(n_bo)

        cos_beta = n_bo.dot(n_bc)
        v_b = n_bo*cos_beta - n_bc
        
        v_m = np.array([0., 0., 0.])
        v_m[0] = self.vx
        
        # 负号是因为相机旋转了180
        v_m[1] = -self.vy*v_b[1]
        v_m[2] = -self.vz*v_b[2]
        
        v = saturation(v_m, 10)

        return v
    

    # 给FLU的速度
    def frd_velcmd_update(self,F,R,D):
        if F <= 0:
            return self.last_v_cmd
        frd_time_scale = self.frd_time_scale
        # frd_time_scale = self.frd_time_scale if F > 3 else 1
        
        pre_time = F/self.vx*frd_time_scale
        vy = -R/pre_time
        vz = -D/pre_time
        v_m =  np.array([self.vx, vy, vz],dtype=float)
        # print(v_m)      
        v = saturation(v_m, 10)

        return v
