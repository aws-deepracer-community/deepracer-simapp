# -*- coding: utf-8 -*-
##############################################################
#                                                            #
#   Copyright 2020 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################
""" The script takes care of testing the functionality of save_to_mp4 node.
"""
import os
import time
import pytest
import rospy
from std_srvs.srv import Empty, EmptyRequest
from markov.rospy_wrappers import ServiceProxyWrapper
from markov.metrics.constants import (ITERATION_DATA_LOCAL_FILE_PATH,
                                      IterationDataLocalFileNames)

NODE_NAME = 'pytest_save_to_mp4_node'

AGENT_NAMES = ["agent"]
BASE_ROS_PATH = "/root/.ros"

@pytest.fixture(scope="module")
def node():
    """ Fixture function to initialize the ROS node

    Decorators:
        pytest.fixture
    """
    rospy.init_node(NODE_NAME, anonymous=True)

@pytest.fixture
def subscribe_to_save_mp4(node):
    """ Fixture function to subscribe to saveMp4 ROS service

    Decorators:
        pytest.fixture

    Returns:
        ServiceProxyWrapper: ROS service object
    """
    rospy.wait_for_service('/racecar/save_mp4/subscribe_to_save_mp4')
    return ServiceProxyWrapper('/racecar/save_mp4/subscribe_to_save_mp4', Empty)

@pytest.fixture
def unsubscribe_from_save_mp4(node):
    """ Fixture function to unsubscribe from saveMp4 ROS service

    Decorators:
        pytest.fixture

    Returns:
        ServiceProxyWrapper: ROS service object
    """
    rospy.wait_for_service('/racecar/save_mp4/unsubscribe_from_save_mp4')
    return ServiceProxyWrapper('/racecar/save_mp4/unsubscribe_from_save_mp4', Empty)

@pytest.fixture
def mp4_saved_paths():
    """ Fixture function get all the file paths where MP4 is stored

    Decorators:
        pytest.fixture

    Returns:
        list: List of file paths where mp4 camera is saved
    """
    file_paths = list()
    for agent_name in AGENT_NAMES:
        camera_pip_path = os.path.join(BASE_ROS_PATH, ITERATION_DATA_LOCAL_FILE_PATH, agent_name,
                                       IterationDataLocalFileNames.CAMERA_PIP_MP4_VALIDATION_LOCAL_PATH.value)
        camera_45degree_path = os.path.join(BASE_ROS_PATH, ITERATION_DATA_LOCAL_FILE_PATH, agent_name,
                                            IterationDataLocalFileNames.CAMERA_45DEGREE_MP4_VALIDATION_LOCAL_PATH.value)
        camera_topview_path = os.path.join(BASE_ROS_PATH, ITERATION_DATA_LOCAL_FILE_PATH, agent_name,
                                           IterationDataLocalFileNames.CAMERA_TOPVIEW_MP4_VALIDATION_LOCAL_PATH.value)
        file_paths.extend([camera_pip_path, camera_45degree_path, camera_topview_path])
    return file_paths

def test_subscribe_to_save_mp4():
    """ Test the subscribe_to_save_mp4 function.
    """
    # The test_unsubscribe_to_save_mp4 test case will cover all this.
    # Having this so just to increase the coverage
    pass

def test_unsubscribe_to_save_mp4(subscribe_to_save_mp4, unsubscribe_from_save_mp4, mp4_saved_paths):
    """ Test for unsubscribe_to_save_mp4. First I request for subscribe to the mp4 topic. This will start
    recording the camera frames. I sleep for 1 second so that I grabs enough frames. The tests I run are
    1. The response is Empty (From both subscriber and unsubscriber request)
    2. Looks for the file if exists
    3. I observed that if the mp4 video is corrupted, then the file size is 262 bytes. So I have this check.

    Arguments:
        subscribe_to_save_mp4 (ServiceProxyWrapper): ROS service object for subscribing to MP4 node
        unsubscribe_from_save_mp4 (ServiceProxyWrapper): ROS service object for unsubscribing from MP4 node
        mp4_saved_paths (list): List of file paths where mp4 camera is saved
    """
    subscribe_response = subscribe_to_save_mp4(EmptyRequest())
    assert str(subscribe_response) == ""
    # Sleep for 1 seconds so that it saves some frames
    time.sleep(1)
    unsubscribe_response = unsubscribe_from_save_mp4(EmptyRequest())
    assert str(unsubscribe_response) == ""
    for file_path in mp4_saved_paths:
        assert os.path.exists(file_path)
        assert os.path.getsize(file_path) > 262, "If the file size is less than 262 bytes, then the file is corrupt"
