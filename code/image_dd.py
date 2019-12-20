import traceback
import os, sys
from os.path import dirname, abspath
import cv2
import numpy as np
import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import time
from cv_bridge import CvBridge, CvBridgeError
#import imgaug.augmenters as iaa


input_topic = '/mako_1/mako_1/image_raw'
output_topic = '/camera/image_dd'
image_pub = rospy.Publisher(output_topic,Image, queue_size=1)
bridge = CvBridge()

TW = 160
def data_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    H, W, C = cv_image.shape
    TH = int(H // (float(W) / float(TW)))
    #print(H, W, TH, TW)

    resized = cv2.resize(cv_image, dsize=(TW, TH))
    img_msg = bridge.cv2_to_imgmsg(resized, "bgr8")
    image_pub.publish(img_msg)

image_sub = rospy.Subscriber(input_topic,Image, data_callback)

rospy.init_node('image_resize', anonymous=True)
while not rospy.is_shutdown():
    try:
        rospy.spin()
    except Exception as e:
        print('error')
        traceback.print_exc()
        print(repr(e))
        exit(1)
    print('published image correctly!')
    time.sleep(0.1)

exit(0)
