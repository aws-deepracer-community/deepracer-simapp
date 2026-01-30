#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

""" This script is to mock the camera images that we get when racecar is spawned.
"""
import os
import sys
import ament_index_python
# This numpy.core.multiarray is required else cv2 will fail (https://github.com/opencv/opencv/issues/8139)
import numpy.core.multiarray
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImg
from cv_bridge import CvBridge, CvBridgeError
from markov.utils import get_racecar_names

# Get package path
try:
    DEEPRACER_SIM_PATH = ament_index_python.get_package_share_directory('deepracer_simulation_environment')
except:
    # Fallback for development
    DEEPRACER_SIM_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..")

TEST_FIXTURE_NODES_PATH = os.path.join(DEEPRACER_SIM_PATH, "tests")
IMG_PATH = "{0}/test_data/camera_images".format(TEST_FIXTURE_NODES_PATH)
TOTAL_IMAGES = 20

def get_racecar_camera_topics(racecar_name):
    """ The camera topics where the racecar publishes the frames

    Arguments:
        racecar_name (str): The name of the racecar
    """
    return [
        "/{}/main_camera/zed/rgb/image_rect_color".format(racecar_name),
        "/sub_camera/zed/rgb/image_rect_color"
    ]

def get_next_img(img_id):
    """ To fetch the cv2 image. I have added 20 images. The frames are published
    in a cyclic manner.

    Arguments:
        img_id (int): The image id to fetch
    """
    img_name = "img_{}.jpg".format(img_id)
    img_path = os.path.join(IMG_PATH, img_name)
    if os.path.exists(img_path):
        return cv2.imread(img_path)
    else:
        # Return a default black image if test images don't exist
        return cv2.imread(os.path.join(os.path.dirname(__file__), "default_test_image.jpg")) or \
               (255 * np.ones((480, 640, 3), dtype=np.uint8))


class MockCameraNode(Node):
    def __init__(self, racecar_names):
        super().__init__('mock_camera_topic_nodes')
        
        self.bridge = CvBridge()
        self.pub_list = []
        
        # Create publishers for each racecar's camera topics
        for racecar_name in racecar_names:
            camera_topics = get_racecar_camera_topics(racecar_name)
            for topic in camera_topics:
                pub = self.create_publisher(ROSImg, topic, 10)
                self.pub_list.append(pub)
        
        # Create timer to publish at 15Hz
        self.timer = self.create_timer(1.0/15.0, self.publish_frames)
        self.img_id = 0
        
        self.get_logger().info(f'Mock camera node started for {len(racecar_names)} racecars')

    def publish_frames(self):
        """Timer callback to publish camera frames"""
        img = get_next_img(self.img_id)
        self.img_id = (self.img_id + 1) % TOTAL_IMAGES
        
        try:
            frame = self.bridge.cv2_to_imgmsg(img, "bgr8")
            for pub in self.pub_list:
                pub.publish(frame)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        RACER_NUM = int(sys.argv[1]) if len(sys.argv) > 1 else 1
        RACECAR_NAMES = get_racecar_names(RACER_NUM)
        
        node = MockCameraNode(RACECAR_NAMES)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception in mock camera node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
