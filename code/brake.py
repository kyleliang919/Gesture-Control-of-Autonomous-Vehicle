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
                 'steer': (topic_prefix+'steer_cmd', PositionWithSpeed),
                 'enable': (topic_prefix+'enable', Bool)}

pub_enable = rospy.Publisher(*pub_cmd_topic['enable'], queue_size=1)
pub_brake = rospy.Publisher(*pub_cmd_topic['brake'], queue_size=1) # queue_size not handled

def data_callback(data):
    if int(data.num_humans) > 0:
        print(int(data.num_humans), 'humans detected')
        pub_enable.publish(True)
        pub_brake.publish(f64_cmd=0.5, enable=True)
        rospy.loginfo('sent brake command')
        t0 = time.time()
        time.sleep(brake_time)
        pub_brake.publish(f64_cmd=0.0, enable=True)
        rospy.loginfo('cancelled brake command, %.3f sec' % (time.time() - t0))

pose_sub = rospy.Subscriber(input_topic, OpenPoseHumanList, data_callback)

rospy.init_node('brake', anonymous=True)
pub_enable.publish(True)

brake_time = 0.8

while not rospy.is_shutdown():
    try:
        rospy.spin()
    except KeyboardInterrupt:
	pub_enable.publish(False)
        exit(1)
exit(0)
