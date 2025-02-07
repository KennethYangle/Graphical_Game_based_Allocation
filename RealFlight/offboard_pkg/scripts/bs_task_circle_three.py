#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

import io
import os
import time
import numpy as np

import json5

from rgbd_proc import rgbd_img_proc
from visual_servo_bs import visual_servo_bs
from Px4Controller import construct_vel_target

from utils import constrain_rad

file_pwd = os.path.dirname(os.path.abspath(__file__))
time_prefix = time.strftime("%Y-%m-%d_%H-%M-%S")


task_json = "{}/config.json".format(file_pwd)
with io.open(task_json,"r",encoding="utf-8") as fp:
    dataD = json5.load(fp)

circle_posL = dataD["circle_posL"]
vs_dict = dataD["vs_dict"]



debug_nameL = [
    "task_state",
    "yolo_px"   ,
    "yolo_frd"  ,
    "cmdv_flu"  ,
    "allocation"  ,
]

class Debuger:
    def __init__(self):
        self.pub_dict = {}
        for i in debug_nameL:
            self.pub_dict[i] = rospy.Publisher('/bs_debuger/{}'.format(i), String, queue_size=1)

    def update(self,debug_info_dict):
        for k,v in debug_info_dict.items():
            self.pub_dict[k].publish(v)




TASK_ACCESS     = 0
TASK_VISUAL     = 1
TASK_THROUGH    = 2

TASK_ALLOCATION = 11
TASK_SWARM_POS  = 12

def sat(a, maxv):
    n = np.linalg.norm(a)
    if n > maxv:
        return a / n * maxv
    else:
        return a

class TaskCircleThree:
    def __init__(self,px4_control,target_allocater):
        self.px4_control = px4_control
        self.target_allocater = target_allocater
        self.tgt_circle_pos = np.array([0., 0., 0.])
        self.tgt_circle_pos_idx = 0

        self.target_pos_sub = rospy.Subscriber('/allocation/target_pos', Point, self.target_pos_callback)

    def target_pos_callback(self, msg):
        # self.tgt_circle_pos = np.array([msg.x, msg.y, msg.z])
        self.tgt_circle_pos_idx = int(msg.x)
    
    def run(self):
        px4_control = self.px4_control
        target_allocater = self.target_allocater


        # img_proc = rgbd_img_proc()
        visual_servo = visual_servo_bs()

        debuger = Debuger()


        task_state = TASK_ALLOCATION
        mav_id = rospy.get_param('~mav_id', 1)

        print("TaskCircleThree mav_id:{}".format(mav_id))
        
        v_cmd = [0]*3
        tgt_yaw = 0
        visual_servo.reset()
        visual_servo.__dict__.update(vs_dict)

        vel = 0.5
        while True:
            # print("self.tgt_circle_pos_idx:", self.tgt_circle_pos_idx)
            self.tgt_circle_pos = target_allocater.p_search[self.tgt_circle_pos_idx]
            # print("tgt_circle_pos: {}".format(self.tgt_circle_pos))

            if np.all(target_allocater.Pcur[mav_id-1]==0) and np.all(self.tgt_circle_pos==0):
                time.sleep(0.02)
                continue

            self.tgt_circle_pos[0] += 1
            dlt_pos = self.tgt_circle_pos - target_allocater.Pcur[mav_id-1]
            dlt_pos_yz = np.array([0, dlt_pos[1], dlt_pos[2]])

            tag_vel_yz = sat(dlt_pos_yz * 3.0, 3.0)
            tag_vel_xyz = dlt_pos/np.linalg.norm(dlt_pos)*vel
            px4_control.command_vel = construct_vel_target(tag_vel_xyz[0], tag_vel_yz[1], tag_vel_yz[2], frame="ENU")
            
            if np.linalg.norm(dlt_pos) < 0.3:
                for i in range(10):
                    px4_control.moveByVelocityYawrateENU(vx=0.5)
                    time.sleep(0.1)
                    print("Through")
                break
            time.sleep(0.02)


