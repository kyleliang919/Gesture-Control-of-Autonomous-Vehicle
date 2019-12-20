import traceback
import os, sys
from os.path import dirname, abspath
import cv2
import numpy as np
import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool
from openpose_ros_msgs.msg import OpenPoseHumanList
import time
input_topic = '/openpose_ros/human_list'
def data_callback(data):
    #print(data.num_humans)
    #print(type(data.human_list[0].left_hand_key_points_with_prob[0].x))
    #print(dir(data.human_list[0]))
    center = data.human_list[0].body_key_points_with_prob[1]
    left_hand = data.human_list[0].body_key_points_with_prob[7] 
    right_hand =  data.human_list[0].body_key_points_with_prob[4]
    print("right hand x: " + str(right_hand.x))
    print("right hand y: " + str(right_hand.y))
    print("left hand x: " + str(left_hand.x))
    print("left hand y: " + str(left_hand.y))
    print("center x: " + str(center.x))
    print("center y: " + str(center.y))
    left = left_hand.y - center.y
    right = right_hand.y - center.y
    if right_hand.x - left_hand.x > 0:
        print('cross')
    elif left < 0 and right >  0:
        print('only left hand raised')
    elif right < 0 and left > 0:
        print('only right hand raised')
    elif right <  0 and left < 0:
        print('both left and right hand raised')
    #print(data.human_list[0].body_key_points_with_prob[7])
    #print(data.human_list[0].body_key_points_with_prob[7])
pose_sub = rospy.Subscriber(input_topic, OpenPoseHumanList, data_callback)
rospy.init_node('read_pose', anonymous=True)
while not rospy.is_shutdown():
    try:
        rospy.spin()
    except Exception as e:
        print('error')
        traceback.print_exc()
        print(repr(e))
        exit(1)
    print('published pose correctly!')
    time.sleep(3)
exit(0)
