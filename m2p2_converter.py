#!/usr/bin/env python

import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import CompressedImage, Image, Imu, PointCloud2, NavSatFix
from cv_bridge import CvBridge

# Mapping of original -> new topics
TOPIC_MAP = {
    '/sensor_suite/witmotion_imu/imu': '/kitti/oxts/imu',
    '/sensor_suite/f9p_rover/fix': '/kitti/oxts/gps/fix',
    '/sensor_suite/ouster/points': '/kitti/velo/pointcloud',
}

LEFT_IMAGE_COMPRESSED = '/sensor_suite/left_camera_optical/image_color/compressed'
RIGHT_IMAGE_COMPRESSED = '/sensor_suite/right_camera_optical/image_color/compressed'
LEFT_IMAGE_GRAY = '/kitti/camera_gray_left/image_raw'
RIGHT_IMAGE_GRAY = '/kitti/camera_gray_right/image_raw'

def convert_bag(input_bag):
    bridge = CvBridge()
    output_bag = 'converted_' + os.path.basename(input_bag)
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == LEFT_IMAGE_COMPRESSED or topic == RIGHT_IMAGE_COMPRESSED:
                # Convert compressed image to gray raw image
                try:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg)
                    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                    img_msg = bridge.cv2_to_imgmsg(gray_image, encoding="mono8")
                    img_msg.header = msg.header
                    new_topic = LEFT_IMAGE_GRAY if topic == LEFT_IMAGE_COMPRESSED else RIGHT_IMAGE_GRAY
                    outbag.write(new_topic, img_msg, t)
                except Exception as e:
                    rospy.logwarn("Failed to convert image: {}".format(e))
            elif topic in TOPIC_MAP:
                outbag.write(TOPIC_MAP[topic], msg, t)
            else:
                outbag.write(topic, msg, t)

    print("✅ Done. Saved to: {}".format(output_bag))

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Usage: python m2p2_converter.py m2p2.bag")
        sys.exit(1)
    convert_bag(sys.argv[1])


