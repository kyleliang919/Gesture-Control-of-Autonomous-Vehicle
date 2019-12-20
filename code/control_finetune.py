import os, sys
from os.path import dirname, abspath
import time

likely_lib_path = os.path.join(dirname(dirname(dirname(abspath(__file__)))), 'lib', 'p27_site_packages')
if os.path.exists(likely_lib_path):
    sys.path.insert(0, likely_lib_path)

import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from openpose_ros_msgs.msg import OpenPoseHumanList
from std_msgs.msg import Bool, Float64
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

accel_cmd = PacmodCmd()
prev_time = time.time()
desired_speed = 0.0
def speed_control(data):
    current_time = time.time()
    global accel_cmd
    global prev_time
    global dersired_speed
    current_time = time.time()
    delta_time = current_time - prev_time
    
    current_error = desired_speed - data.data
    #print ("desired speed: " + str(desired_speed))
    if not (desired_speed == 0 and data.data == 0):
        if current_error > -0.01 and delta_time > 3.0:
            output = 0.36
            accel_cmd.f64_cmd = output
            pub_accel.publish(accel_cmd)
        else:
            if delta_time > 3.0:
                prev_time = current_time  # update previous time after the receive the desired speed
            output = 0.31
            accel_cmd.f64_cmd = output
            pub_accel.publish(accel_cmd)


key = ''
def data_callback(data):
    if len(data.human_list) == 0:
        return
    global key
    center = data.human_list[0].body_key_points_with_prob[1]
    left_hand = data.human_list[0].body_key_points_with_prob[7]
    right_hand =  data.human_list[0].body_key_points_with_prob[4]
    right_elbow = data.human_list[0].body_key_points_with_prob[3]
    left_elbow = data.human_list[0].body_key_points_with_prob[6]

    center_exist = (center.x != 0 and center.y !=0)
    left_hand_exist = left_hand.x !=0 and left_hand.y != 0
    right_hand_exist = right_hand.x !=0 and right_hand.y != 0
    right_elbow_exist = right_elbow.x !=0 and right_elbow.y != 0
    left_elbow_exist = left_elbow.x !=0 and left_elbow.y != 0
    #if center.x == 0 and center.y == 0:
    #    print("drop frame because no center")
    #    key = ''
    #    return 
    #if left_hand.x == 0 and left_hand.y == 0:
    #    print("drop frame because no left hand")
    #    key = ''
    #    return
    #if right_hand.x == 0 and right_hand.y == 0:
    #    print("drop frame because no right hand")
    #    key = ''
    #    return
    #if right_elbow.x == 0 and right_elbow.y == 0:
    #    print("drop frame because no right elbow")
    #    key = ''
    #    return
    #if left_elbow.x == 0 and left_elbow.y == 0:
    #    print("drop frame because no left elbow")
    #    key = ''
    #    return

    right_elbow_out = right_elbow_exist and right_hand_exist and (right_elbow.x - right_hand.x) < -5
    left_elbow_out = left_elbow_exist and left_hand_exist and (left_elbow.x - left_hand.x) > 5
    left_hand_raise = left_hand_exist and center_exist and (left_hand.y - center.y) < -10
    right_hand_raise = right_hand_exist and center_exist and (right_hand.y - center.y) < -10
    crossing = left_hand_exist and right_hand_exist and (right_hand.x - left_hand.x) > 15
    close_hand = left_hand_exist and right_hand_exist and (left_hand.x - right_hand.x) > 0 and  (left_hand.x - right_hand.x) <= 20 


    #if crossing and left_hand_raise and right_hand_raise:
        #print('high cross')
        #print("setting speed to 1")
        #print(right_hand.x - left_hand.x)
        #key = 'sp1'
        #pub_enable.publish(True)
        #pub_gear.publish(ui16_cmd=des_gear, enable = True, ignore = False, clear = False)
    if crossing:
        print('low cross')
        print("setting speed to 0.5")
        print(right_hand.x - left_hand.x)
        key = 'sp0.5'
        #pub_enable.publish(True)
        #pub_gear.publish(ui16_cmd=des_gear, enable = True, ignore = False, clear = False)
    elif left_hand_raise and not right_hand_raise:
        print('only left hand raised')
        key = 'left_raise_only'
    elif right_hand_raise and not left_hand_raise:
        key = 'right_raise_only'
        print('only right hand raised')
    elif left_hand_raise and right_hand_raise and left_elbow_out and right_elbow_out: # crossing is already eliminated
	    #print (right)
        print('both left and right hand raised, stop')
        key = 'stop'
    elif close_hand:
        print("close hand")
        print("setting speed to 1")
        key = "sp1"
    else:
        #print("x cord")
        #print(right_hand.x - left_hand.x)
        #print("y cord")
        #print(right_hand.y - left_hand.y)
        key = ''
        print('no command detected')

rospy.init_node('gesture_ctrl', anonymous=True)
pose_sub = rospy.Subscriber(input_topic, OpenPoseHumanList, data_callback)
speed_sub = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, speed_control)


brake_time = 0.5
signal_time = 2
steer_time = .5
accel_time = 2.0
turn_mag = 0
turn_increase = 2.0 
pub_enable.publish(True)
pub_gear.publish(ui16_cmd=PacmodCmd.SHIFT_FORWARD, enable=True,ignore=False,clear = False)
while not rospy.is_shutdown():
    try:
        if key == 'stop':
            pub_enable.publish(True)
            pub_brake.publish(f64_cmd=0.5, enable=True)
            rospy.loginfo('sent brake command')
            t0 = time.time()
            time.sleep(brake_time)
            pub_brake.publish(f64_cmd=0.0, enable=True)
            desired_speed = 0.0
            rospy.loginfo('cancelled brake command, %.3f sec' % (time.time() - t0))

        # elif key in ('j', 'k'):
        #     pub_enable.publish(True)
        #     sig_dir, ui16_cmd = {'j': ('left', PacmodCmd.TURN_LEFT),
        #                          'k': ('right', PacmodCmd.TURN_RIGHT)}[key]
        #     pub_turn.publish(ui16_cmd=ui16_cmd, enable=True)
        #     rospy.loginfo('sent turn '+sig_dir+' command')
        #     t0 = time.time()
        #     time.sleep(signal_time)
        #     pub_turn.publish(ui16_cmd=PacmodCmd.TURN_NONE, enable=True)
        #     rospy.loginfo('cancelled turn %s command, %.3f sec' % (sig_dir, time.time() - t0))

        elif key in ('left_raise_only', 'right_raise_only'):
            pub_enable.publish(True)
            pub_brake.publish(f64_cmd=0.0, enable=True)
            steer_dir, th, w = {'right_raise_only': ('left', -turn_increase, +1.8),
                                'left_raise_only': ('right', turn_increase, +1.8)}[key]
            #steer_dir, th, w = {'right_raise_only': ('left',  -1.0, +0.2),
            #                    'left_raise_only': ('right', +1.0, +0.2)}[key]
            turn_mag += th
            if turn_mag > 0:
                turn_mag = min(8.0, turn_mag)
            elif turn_mag < 0:
                turn_mag = max(-8.0, turn_mag)
            steer_cmd = PositionWithSpeed()
            steer_cmd.angular_position = turn_mag 
            steer_cmd.angular_velocity_limit = w
            #pub_steer.publish(angular_position=th, angular_velocity_limit=w, enable=True)
            pub_steer.publish(steer_cmd)
            rospy.loginfo('sent steer '+steer_dir+' command')
            t0 = time.time()
            time.sleep(steer_time)
            rospy.loginfo('ended steer %s command, %.3f sec' % (steer_dir, time.time() - t0))

        elif key == 'sp1':
            desired_speed = 1.0
            pub_enable.publish(True)
            pub_brake.publish(f64_cmd=0.0, enable=True)
            #pub_accel.publish(f64_cmd=0.4, enable=True)
            #rospy.loginfo('sent accelerate command')
            #t0 = time.time()
            #time.sleep(accel_time)
            #pub_accel.publish(f64_cmd=0.0, enable=True)
            #rospy.loginfo('cancelled accelerate command, %.3f sec' % (time.time() - t0))
        elif key == 'sp0.5':
            pub_enable.publish(True)
            pub_brake.publish(f64_cmd=0.0, enable=True)
            desired_speed = 0.5
        else:
            key = ''
    except KeyboardInterrupt:
        pub_enable.publish(False)
        exit(1)
