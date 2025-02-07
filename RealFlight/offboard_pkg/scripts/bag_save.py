#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from mavros_msgs.msg import State


import os
import time

import subprocess
import signal

'''
解锁开始记录bag
上锁停止记录bag
'''


file_pwd = os.path.dirname(os.path.abspath(__file__))
bag_path = "{}/../../../bag".format(file_pwd)
time_prefix = time.strftime("%Y-%m-%d_%H-%M-%S")

if not os.path.exists(bag_path):
    os.mkdir(bag_path)


d435i_topic     = "d435i/color/image_raw d435i/aligned_depth_to_color/image_raw"
mavros_topic    = "mavros/local_position/pose mavros/local_position/velocity_body mavros/local_position/velocity_local"
ctrl_topic      = "mavros/setpoint_raw/local mavros/setpoint_raw/attitude"




debug_nameL = [
    "task_state",
    "yolo_px"   ,
    "yolo_frd"  ,
    "cmdv_flu"  ,
]

debug_nameL.append("image_res")
debug_topicL = ["bs_debuger/{}".format(i) for i in debug_nameL]

debug_topic = " ".join(debug_topicL)




def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)


class BagSaver:
    def __init__(self):
        self.arm_state = False
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)

    def mavros_state_callback(self, msg):
        self.arm_state = msg.armed
    
    def run(self):
        # 等待无人机解锁
        while not self.arm_state:
            time.sleep(0.1)
        
        # 开始记录数据
        save_proc = subprocess.Popen(rosbag_cmd,shell=True,cwd=bag_path)
        pid = save_proc.pid
        # 等待无人机上锁
        while self.arm_state:
            time.sleep(1)
        # time.sleep(15)
        terminate_ros_node("/record")











if __name__=="__main__":
    rospy.init_node("bag_save_node")

    mav_num = rospy.get_param('mav_num', 10)
    dds_topicL = ["/vicon/AA_rfly_{:01d}/pose".format(i+1) for i in range(10)]
    dds_topic = " ".join(dds_topicL)
    circle_topicL = ["/vicon/AA_circle_{:01d}/pose".format(i+1) for i in range(5)]
    circle_topic = " ".join(circle_topicL)

    save_topic = "{} {} {} {} {} {}".format(d435i_topic,mavros_topic,ctrl_topic,debug_topic,dds_topic,circle_topic)

    rosbag_cmd = "rosbag record {}".format(save_topic)
    
    print(os.path.abspath(bag_path))
    print("save_topic:")
    print(save_topic.replace(" ","\n"))

    bag_saver = BagSaver()
    bag_saver.run()



    
 


    