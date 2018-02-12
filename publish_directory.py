#!/usr/bin/python

import argparse
import cv2
import glob
import sys
import rospy
from sensor_msgs.msg import Image
import cv_bridge


parser = argparse.ArgumentParser()
parser.add_argument('dir')
parser.add_argument('--regex', default='left_*.png', type=str,
        help='The file regex to match for playback.')

args = parser.parse_args()

rospy.init_node('image_publisher')

pub = rospy.Publisher('camera/left/undistorted', Image, queue_size=10)

bridge = cv_bridge.CvBridge()
r = rospy.Rate(2)

names = glob.glob('{}/left_*.png'.format(args.dir))
names.sort()
for fname in names:
    rospy.loginfo('Publishing {}'.format(fname))
    img = cv2.imread(fname)
    pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    r.sleep()

    if rospy.is_shutdown():
        break
