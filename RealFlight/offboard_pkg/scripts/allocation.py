#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Point, PoseStamped 


import io
import os
import time



import numpy as np
from scipy.optimize import linear_sum_assignment




file_pwd = os.path.dirname(os.path.abspath(__file__))
time_prefix = time.strftime("%Y-%m-%d_%H-%M-%S")

import sys
__BASE__ = os.path.dirname(os.path.dirname(file_pwd))
sys.path.append(__BASE__+"/Graphical")
from main_ly import useGTA


class Allocation:
    def __init__(self, mav_id):
        self.mav_num = rospy.get_param('mav_num', 10)
        circle_num = 3
        self.mav_id = mav_id
        self.mav_posL = np.zeros((self.mav_num,2))

        self.last_mav_num = 0
        self.last_circle_num = 0

        self.circle_posL = np.zeros((self.mav_num,2))
        self.Pcur = np.zeros((self.mav_num,3))
        self.p_search = np.zeros((self.mav_num,3))
        self.ros_subL = [
            rospy.Subscriber("/vrpn_client_node/BUAA_{:02d}/pose".format(i+1), PoseStamped, self.mav_pos_callback,(i,))
            for i in range(self.mav_num)    
        ]
        self.ros_circle_subL = [
            rospy.Subscriber("/vrpn_client_node/CIRCLE_{:02d}/pose".format(i+1), PoseStamped, self.circle_pos_callback,(i,))
            for i in range(circle_num)
        ]
        self.timer = rospy.Timer(rospy.Duration(2), self.allocate_callback)

        self.target_pos_pub =  rospy.Publisher('/allocation/target_pos', Point, queue_size=1)
        
    
    def mav_pos_callback(self,msg,i):
        # self.mav_posL[i,:] = msg.pose.position.x, msg.pose.position.y
        self.Pcur[i,:] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

    def circle_pos_callback(self,msg,i):
        # self.circle_posL[i,:] = msg.pose.position.x, msg.pose.position.y
        self.p_search[i,:] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    
    def allocate(self,circle_posL):
        print("mav_posL:")
        print(self.mav_posL)
        cost_mat = [
            [np.linalg.norm(mav_pos-circle_pos) for circle_pos in circle_posL]
            for mav_pos in self.mav_posL
        ]
        _,task_result = linear_sum_assignment(cost_mat)

        return task_result

    def allocate_yk(self):
        print("Pcur:", self.Pcur)
        print("p_search:", self.p_search)
        N = max(np.size(self.Pcur, 0), np.size(self.p_search, 0))
        ViewR = np.array([4000 for i in range(N)])
        p_next = useGTA(self.Pcur, ViewR, self.p_search)
        print("p_next:", p_next)

        return p_next[self.mav_id-1]

    def allocate_callback(self, event):
        cnt_zero_line_of_Pcur = 0
        cnt_zero_line_of_p_search = 0
        for i in range(self.mav_id-1):
            if np.all(self.Pcur[i]==0):
                cnt_zero_line_of_Pcur += 1

        local_to_self_p_search_dic = {}
        for i in range(self.p_search.shape[0]):
            local_to_self_p_search_dic[i-cnt_zero_line_of_p_search] = i
            if np.all(self.p_search[i]==0):
                cnt_zero_line_of_p_search += 1

        Pcur = self.Pcur[[not np.all(self.Pcur[i]==0) for i in range(self.Pcur.shape[0])], :]
        p_search = self.p_search[[not np.all(self.p_search[i]==0) for i in range(self.p_search.shape[0])], :]

        # print("Pcur:", Pcur)
        # print("p_search:", p_search)

        this_mav_num = np.size(Pcur, 0)
        this_circle_num = np.size(p_search, 0)
        # event-trigger, only re-allocate when the number of mavs or targets changed.
        if (this_mav_num != self.last_mav_num) or (this_circle_num != self.last_circle_num):
            time_start = time.time()
            print("Pcur:", Pcur)
            print("p_search:", p_search)
            N = max(this_mav_num, this_circle_num)
            ViewR = np.array([4000 for i in range(N)])
            p_next = useGTA(Pcur, ViewR, p_search)
            print("p_next:", p_next)

            tgt_idx = local_to_self_p_search_dic[p_next[self.mav_id-1-cnt_zero_line_of_Pcur]]

            target_pos = Point()
            # target_pos.x = p_next[self.mav_id-1-cnt_zero_line_of_Pcur][0]
            # target_pos.y = p_next[self.mav_id-1-cnt_zero_line_of_Pcur][1]
            # target_pos.z = p_next[self.mav_id-1-cnt_zero_line_of_Pcur][2]
            target_pos.x = tgt_idx
            self.target_pos_pub.publish(target_pos)
            print("target_pos:", self.p_search[int(target_pos.x)])

            time_end = time.time()
            print("Total time: {} ms".format((time_end - time_start) * 2000))

        self.Pcur = np.zeros((self.mav_num,3))
        self.p_search = np.zeros((self.mav_num,3))

        self.last_mav_num = this_mav_num
        self.last_circle_num = this_circle_num
