#!/bin/bash
source catkin_kl2/devel/setup.sh
python code/image_dd.py &
roslaunch openpose_ros openpose_ros.launch
