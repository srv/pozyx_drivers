
printf "\n\n---Go to the folder containing the bagfiles---\n---------prior executing this script--------\n\n" &

x=$1
y=${x%.bag}

z=$PWD

# echo $z/position${y##*/}.txt

roslaunch uwb_methods anchorInfoFilter.launch &

roslaunch uwb_methods anchorRangeMatcher.launch &

rostopic echo -p /pozyx_node/pose/pose/position > $z/${y##*/}_POZYX.txt &

rostopic echo -p /anchor_range_matcher/pose/pose/position > $z/${y##*/}_ICP.txt &

rosbag play $1 --clock --pause