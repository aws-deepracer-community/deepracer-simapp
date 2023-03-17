from typing import Any, Callable, Optional, Iterable
from unittest import TestCase
from unittest.mock import patch, MagicMock, call
import inspect

from deepsim.ros.ros_util import ROSUtil
from deepsim.exception import DeepSimException

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class ROSUtilTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_is_ros_node_alive(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            assert ROSUtil.is_ros_node_alive(myself())
            assert not ROSUtil.is_ros_node_alive(myself() + "_ghost")

    def test_wait_for_rosnode_alive_nodes(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            ROSUtil.wait_for_rosnode(alive_nodes=[myself()])
            assert rosnode_mock.get_node_names.call_count == 1
            time_mock.sleep.assert_not_called()

    def test_wait_for_rosnode_alive_nodes_error(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            node_name = myself() + "_ghost"
            with self.assertRaises(DeepSimException):
                ROSUtil.wait_for_rosnode(alive_nodes=[node_name],
                                         max_retry_attempts=3,
                                         backoff_time_sec=0.5)
            assert rosnode_mock.get_node_names.call_count == 1 + 3
            time_mock.sleep.has_calls(
                call(0.5), call(0.5), call(0.5), call(0.5), call(0.5)
            )

    def test_wait_for_rosnode_dead_nodes(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            node_name = myself() + "_dead"
            ROSUtil.wait_for_rosnode(dead_nodes=[node_name])
            assert rosnode_mock.get_node_names.call_count == 1
            time_mock.sleep.assert_not_called()

    def test_wait_for_rosnode_dead_nodes_error(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            node_name = myself()
            with self.assertRaises(DeepSimException):
                ROSUtil.wait_for_rosnode(dead_nodes=[node_name],
                                         max_retry_attempts=3,
                                         backoff_time_sec=0.5)
            assert rosnode_mock.get_node_names.call_count == 1 + 3
            time_mock.sleep.has_calls(
                call(0.5), call(0.5), call(0.5), call(0.5), call(0.5)
            )
