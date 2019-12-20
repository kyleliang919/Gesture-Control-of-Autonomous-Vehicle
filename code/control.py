import os, sys
from os.path import dirname, abspath

likely_lib_path = os.path.join(dirname(dirname(dirname(abspath(__file__)))), 'lib', 'p27_site_packages')
if os.path.exists(likely_lib_path):
    sys.path.insert(0, likely_lib_path)

import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from openpose_ros_msgs.msg import OpenPoseHumanList
from std_msgs.msg import Bool
import time
input_topic = '/openpose_ros/human_list'
topic_prefix = '/pacmod/as_rx/'
pub_cmd_topic = {'brake': (topic_prefix+'brake_cmd', PacmodCmd),
                 'accel': (topic_prefix+'accel_cmd', PacmodCmd),
                 'turn': (topic_prefix+'turn_cmd', PacmodCmd),
                 'gear': (topic_prefix+'shift_cmd', PacmodCmd),
                 'steer': (topic_prefix+'steer_cmd', PositionWithSpeed),
                 'enable': (topic_prefix+'enable', Bool)}

pub_enable = rospy.Publisher(*pub_cmd_topic['enable'], queue_size=1)
pub_brake = rospy.Publisher(*pub_cmd_topic['brake'], queue_size=1) # queue_size not handled
pub_accel = rospy.Publisher(*pub_cmd_topic['accel'], queue_size=1)
pub_turn = rospy.Publisher(*pub_cmd_topic['turn'], queue_size=1)
pub_steer = rospy.Publisher(*pub_cmd_topic['steer'], queue_size=1)
pub_gear = rospy.Publisher(*pub_cmd_topic['gear'], queue_size = 1)
des_gear = PacmodCmd.SHIFT_FORWARD

key = ''
def data_callback(data):
    if len(data.human_list) == 0:
        return
    global key
    center = data.human_list[0].body_key_points_with_prob[1]
    left_hand = data.human_list[0].body_key_points_with_prob[7]
    right_hand =  data.human_list[0].body_key_points_with_prob[4]
    left = left_hand.y - center.y
    right = right_hand.y - center.y
    if right_hand.x - left_hand.x > 0:
        key = 'w'
        #pub_enable.publish(True)
        #pub_gear.publish(ui16_cmd=des_gear, enable = True, ignore = False, clear = False)
        print('cross')
    elif left < 0 and right >  0:
        print('only left hand raised')
        key = 'd'
    elif right < 0 and left > 0:
        key = 'a'
        print('only right hand raised')
    elif right <  0 and left < 0:
        key = 'b'
        print('both left and right hand raised')
    else:
        key = ''
        print('no command detected')
pose_sub = rospy.Subscriber(input_topic, OpenPoseHumanList, data_callback)
rospy.init_node('gesture_ctrl', anonymous=True)

brake_time = 0.5
signal_time = 2
steer_time = .5
accel_time = 2.0
pub_enable.publish(True)
pub_gear.publish(ui16_cmd=PacmodCmd.SHIFT_FORWARD, enable=True,ignore=False,clear = False)
while not rospy.is_shutdown():
    try:
        if key == 'b':
            pub_enable.publish(True)
            pub_brake.publish(f64_cmd=0.5, enable=True)
            rospy.loginfo('sent brake command')
            t0 = time.time()
            time.sleep(brake_time)
            pub_brake.publish(f64_cmd=0.0, enable=True)
            rospy.loginfo('cancelled brake command, %.3f sec' % (time.time() - t0))

        elif key in ('j', 'k'):
            pub_enable.publish(True)
            sig_dir, ui16_cmd = {'j': ('left', PacmodCmd.TURN_LEFT),
                                 'k': ('right', PacmodCmd.TURN_RIGHT)}[key]
            pub_turn.publish(ui16_cmd=ui16_cmd, enable=True)
            rospy.loginfo('sent turn '+sig_dir+' command')
            t0 = time.time()
            time.sleep(signal_time)
            pub_turn.publish(ui16_cmd=PacmodCmd.TURN_NONE, enable=True)
            rospy.loginfo('cancelled turn %s command, %.3f sec' % (sig_dir, time.time() - t0))

        elif key in ('a', 'd'):
            pub_enable.publish(True)
            steer_dir, th, w = {'a': ('left',  -0.2, +0.1),
                                'd': ('right', +0.2, +0.1)}[key]
            steer_cmd = PositionWithSpeed()
            steer_cmd.angular_position = th
            steer_cmd.angular_velocity_limit = w
            #pub_steer.publish(angular_position=th, angular_velocity_limit=w, enable=True)
            pub_steer.publish(steer_cmd)
            rospy.loginfo('sent steer '+steer_dir+' command')
            t0 = time.time()
            time.sleep(steer_time)
            rospy.loginfo('ended steer %s command, %.3f sec' % (steer_dir, time.time() - t0))

        elif key == 'w':
            pub_enable.publish(True)
            pub_accel.publish(f64_cmd=0.4, enable=True)
            rospy.loginfo('sent accelerate command')
            t0 = time.time()
            time.sleep(accel_time)
            pub_accel.publish(f64_cmd=0.0, enable=True)
            rospy.loginfo('cancelled accelerate command, %.3f sec' % (time.time() - t0))
        else:
            key = ''
    except KeyboardInterrupt:
        pub_enable.publish(False)
        exit(1)
