#rostopic echo -p /pozyx_node/pose/pose > expUWB/position$1.txt &
#rosbag record -a -O expUWB/$1

#rosbag record -e "/pozyx_node/(.*)" -O expUWB/$1

cd /home/srv/expUWB
rosbag record -e "/pozyx_node/(.*)" -o $1
cd -

