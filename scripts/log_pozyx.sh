
rostopic echo -p /pozyx_node/pose/pose > expUWB/position$1.txt &
rosbag record -a -O expUWB/$1