#!/bin/bash

SH_DIR=$(dirname $0)
WS_DIR="${SH_DIR}/../.."

# 集群参数
mav_id=6
# mav_id=`expr ${HOSTNAME:4:2} + 0`
mav_num=10

echo "mav_id:${mav_id} mav_num:${mav_num}"
sleep 10s

gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200";exec bash"
sleep 5s

gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch mocap_pose mocap_vrpn.launch;exec bash"
sleep 5s

gnome-terminal --window -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_main.launch mav_id:=${mav_id} mav_num:=${mav_num};exec bash"
sleep 5s
