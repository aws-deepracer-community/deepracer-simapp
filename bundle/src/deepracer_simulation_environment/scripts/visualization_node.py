#!/usr/bin/env python3
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

'''This module will launch a ROS node that will have publish visualization topics.
'''

import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
import sys
from sensor_msgs.msg import Image as sensor_image
from deepracer_simulation_environment.msg import AgentRewardData 
from cv_bridge import CvBridge, CvBridgeError
import cv2
import logging
from markov.log_handler.logger import Logger
from markov import utils

logger = Logger(__name__, logging.INFO).get_logger()

class RewardDistributionBarGraph(Node):
    def __init__(self, racecar_name, title="Reward Distribution Bar Graph"):
        super().__init__(f'visualization_node_{racecar_name}')
        self.racecar_name = racecar_name
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            AgentRewardData,
            f"/reward_data/{self.racecar_name}",
            self.reward_data_subscriber,
            QoSProfile(depth=10)
        )
        self.reward_distribution_graph_publisher = self.create_publisher(
            sensor_image, 
            f'/visualization/reward_bar_graph/{self.racecar_name}', 
            10
        )
        self.bridge = CvBridge()
        
        # Declare and get parameters
        self.declare_parameter('vis_number_of_past_frames', 10)
        self.declare_parameter('vis_figure_width', 12)
        self.declare_parameter('vis_figure_height', 6)
        
        self.num_of_past_frames = self.get_parameter('vis_number_of_past_frames').value
        figure_width = self.get_parameter('vis_figure_width').value
        figure_height = self.get_parameter('vis_figure_height').value
        
        # TODO: Change from list to queue
        self.reward_list = [0 for i in range(self.num_of_past_frames)]
        self.action_index_list = [0 for i in range(self.num_of_past_frames)]
        self.image_list = [np.zeros((120, 160)) for i in range(self.num_of_past_frames)]
        self.fig = plt.figure(figsize=(int(figure_width), int(figure_height)))

        # Define the plot specifics
        gs = gridspec.GridSpec(6, self.num_of_past_frames)
        self.image_axes = []
        for i in range(self.num_of_past_frames):
            ax = self.fig.add_subplot(gs[0, i])
            ax.set_xticks([]) 
            ax.set_yticks([]) 
            im_ax = ax.imshow(self.image_list[i])
            self.image_axes.append(im_ax)

        self.reward_axes = self.fig.add_subplot(gs[1, :])
        self.reward_axes.set_xticks([])
        self.reward_axes.set_ylabel('Reward')
        self.reward_line, = self.reward_axes.plot(self.reward_list)

        frame_count = np.arange(-self.num_of_past_frames+1, 1)

        self.action_index_axes = self.fig.add_subplot(gs[2:, :])
        self.action_index_axes.set_xlabel('Past frame count')
        self.action_index_axes.set_ylabel('Action')
        self.action_index_axes.set_xticks(frame_count)
        self.action_bar_rects = self.action_index_axes.bar(frame_count, self.action_index_list, alpha=0.5)

    def reward_data_subscriber(self, data):
        action_index = data.action
        reward_value = data.reward
        action_space_len = data.action_space_len
        image = data.image
        if image is None:
            image = np.zeros((120, 160))
        else:
            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        if not reward_value:
            reward_value = 0.0
        if not action_index:
            action_index = 0

        self.image_list.pop(0)
        self.image_list += [image]

        self.reward_list.pop(0)
        self.reward_list += [reward_value]

        self.action_index_list.pop(0)
        self.action_index_list += [action_index+1]

        for i in range(self.num_of_past_frames):
            self.image_axes[i].set_data(self.image_list[i])
        min_reward =  min(self.reward_list)
        max_reward = max(self.reward_list)
        self.reward_axes.set_ylim((min_reward - 0.1*min_reward, max_reward + 0.1*max_reward))
        self.reward_line.set_ydata(self.reward_list)

        action_indices = np.arange(0, action_space_len+1)
        action_space_names = ['--']
        for i in range(action_space_len):
            action_space_names.append(data.steering_angle_list[i] + '/' + data.speed_list[i])

        self.action_index_axes.set_ylim((0, action_space_len+2))
        self.action_index_axes.set_yticks(action_indices)
        self.action_index_axes.set_yticklabels(action_space_names)

        for rect, h in zip(self.action_bar_rects, self.action_index_list):
            rect.set_height(h)

        self.fig.canvas.draw()

        data = np.fromstring(self.fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        data = data.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))

        self.reward_distribution_graph_publisher.publish(self.bridge.cv2_to_imgmsg(data, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # comma separated racecar names passed as an argument to the node 
        RACER_NUM = int(sys.argv[1]) if len(sys.argv) > 1 else 1
        racecar_names = utils.get_racecar_names(RACER_NUM)
        
        # Create nodes for each racecar
        nodes = []
        for racecar_name in racecar_names:
            node = RewardDistributionBarGraph(racecar_name)
            nodes.append(node)
        
        # Use MultiThreadedExecutor to handle multiple nodes
        executor = MultiThreadedExecutor()
        
        for node in nodes:
            executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            for node in nodes:
                node.destroy_node()
            
    except Exception as ex:
        logger.error(f"Exception in visualization node: {ex}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
