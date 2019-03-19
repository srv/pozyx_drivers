
rostopic echo -p /pozyx_node_A/pose/pose > expUWB/positionA$1.txt &
rostopic echo -p /pozyx_node_B/pose/pose > expUWB/positionB$1.txt &
rosbag record -a -O expUWB/$1