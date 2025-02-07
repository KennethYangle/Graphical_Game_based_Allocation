#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2022-10-18 16:08:35
# @Author  : BrightSoul (653538096@qq.com)

import rospy
import rospkg


import io
import os
import json5
import time

from Px4Controller import Px4Controller


from allocation import Allocation


from bs_task_circle_three import TaskCircleThree





file_pwd = os.path.dirname(os.path.abspath(__file__))
task_json = "{}/config.json".format(file_pwd)
with io.open(task_json,"r",encoding="utf-8") as fp:
    dataD = json5.load(fp)

pos_swarm_initL = dataD["pos_swarm_initL"]





if __name__=="__main__":
    rospy.init_node("offboard_node")
    
    mav_id = rospy.get_param('~mav_id', 1)

    
    pos_swarm_init = pos_swarm_initL[mav_id-1]
    print("mav_id:{} pos_swarm_init:{}".format(mav_id,pos_swarm_init))
    
    # px4_control = Px4Controller(pos_swarm_init=pos_swarm_init)
    px4_control = Px4Controller()
    target_allocater = Allocation(mav_id)



    bs_task = TaskCircleThree(px4_control,target_allocater)

    print("Px4 Controller Initialized!")

    # 开始推位置
    px4_control.start_pub()
    
    # 先检查是否能进入offboard
    # 切换至position模式
    # ch7为高则进入任务模式
    while not px4_control.task_ready:
        time.sleep(0.1)
    print("==============================")
    print("Begin Task!")
    

    # 先起飞然后再执行其他的任务
    # 起飞函数会先解锁然后切换至offboard模式
    px4_control.takeoff(vz=1.25,h=1.25)
    time.sleep(2)


    # 执行一些奇奇怪怪的任务 
    # ==============================================
    
    bs_task.run()


    # ==============================================




    # 降落
    px4_control.moveByVelocityYawrateBodyFrame(0, 0, -0.5, 0.)
    


