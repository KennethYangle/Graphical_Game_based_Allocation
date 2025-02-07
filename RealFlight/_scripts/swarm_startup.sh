#!/bin/bash

SH_DIR=$(dirname $0)
WS_DIR="${SH_DIR}/../.."

RUN_DATE=`date +%Y%m%d_%H%M%S`

# 集群参数
mav_id=3
# mav_id=`expr ${HOSTNAME:4:2} + 0`
mav_num=3

# 每行3个，间隔2m
row_unit=3
offset=2

init_x=$[ -(mav_id-1)/row_unit*offset ]
init_y=$[  (mav_id-1)%row_unit*offset ]


echo "mav_id:${mav_id} mav_num:${mav_num}"
echo "init_x:${init_x} init_y:${init_y}"




sleep 10s

# gnome-terminal -x bash -c "roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS0:921600";exec bash"
gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:115200";exec bash"
sleep 5s

gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_t265_bridge.launch;exec bash"
sleep 5s

gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_d435i_30hz.launch;exec bash"
sleep 5s


gnome-terminal --window -x bash -c "source ${WS_DIR}/devel/setup.bash;roslaunch offboard_pkg bs_main.launch swarm_init_x:=${swarm_init_x} swarm_init_y:=${swarm_init_y};exec bash"

sleep 10s

# gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;rosbag record ${topic} -o ${WS_DIR}/bag/zzfly_real_;exec bash"
gnome-terminal -x bash -c "source ${WS_DIR}/devel/setup.bash;rosrun offboard_pkg bag_save.py;exec bash"
