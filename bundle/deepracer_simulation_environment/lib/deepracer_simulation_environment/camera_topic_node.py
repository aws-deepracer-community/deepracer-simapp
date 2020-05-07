#!/usr/bin/env python
##############################################################
#                                                            #
#   Copyright 2020 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################
""" This script is to mock the camera images that we get when racecar is spawned.
"""
import os
import sys
import rospkg
# This numpy.core.multiarray is required else cv2 will fail (https://github.com/opencv/opencv/issues/8139)
import numpy.core.multiarray
import cv2
import rospy
from sensor_msgs.msg import Image as ROSImg
from cv_bridge import CvBridge, CvBridgeError
from markov.utils import get_racecar_names

rospack = rospkg.RosPack()
DEEPRACER_SIM_PATH = rospack.get_path('deepracer_simulation_environment')

TEST_FIXTURE_NODES_PATH = os.path.join(DEEPRACER_SIM_PATH, "tests")
IMG_PATH = "{0}/test_data/camera_images".format(TEST_FIXTURE_NODES_PATH)
TOTAL_IMAGES = 20

def get_racecar_camera_topics(racecar_name):
    """ The camera topics where the racecar publishes the frames

    Arguments:
        racecar_name (str): The name of the racecar
    """
    return [
        "/{}/deepracer/kvs_stream".format(racecar_name),
        "/{}/main_camera/zed/rgb/image_rect_color".format(racecar_name),
        "/sub_camera/zed/rgb/image_rect_color"
    ]

def get_next_img(img_id):
    """ To fetch the cv2 image. I have added 20 images. The frames are published
    to the Image topic using these images.

    Arguments:
        img_id (int): The id of the image

    Returns:
        Image: cv2 image read from the local file system
    """
    img_path = "{0}/out{1}.png".format(IMG_PATH, img_id)
    return cv2.imread(img_path)

def main(racecar_names):
    """ Read the images in the file and publish these to the image topic. 
    The image topics are the actual topics that racecar agent publishes.
    The images are rotated once all the images are read.

    Arguments:
        racecar_names (str): The name of the racecar
    """
    pub_list = list()
    bridge = CvBridge()
    for racecar_name in racecar_names:
        camera_topics = get_racecar_camera_topics(racecar_name)
        for topic in camera_topics:
            pub_list.append(rospy.Publisher(topic, ROSImg, queue_size=1))

    # Publishing frames at 15Hz
    rate = rospy.Rate(15)
    img_id = 0
    while not rospy.is_shutdown():
        img = get_next_img(img_id)
        img_id = (img_id + 1) % TOTAL_IMAGES
        frame = bridge.cv2_to_imgmsg(img, "bgr8")
        for pub in pub_list:
            pub.publish(frame)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('mock_camera_topic_nodes', anonymous=True)
        RACER_NUM = int(sys.argv[1])
        RACECAR_NAMES = get_racecar_names(RACER_NUM)
        main(RACECAR_NAMES)
    except rospy.ROSInterruptException:
        pass
