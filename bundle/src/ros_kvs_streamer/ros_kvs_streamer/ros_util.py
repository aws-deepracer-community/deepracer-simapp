import time
import rospy
import rospkg
import rosnode

from typing import Optional
from ros_kvs_streamer.exception import KvsException


class ROSUtil:
    """
    ROS Utility Class
    """
    rospack = rospkg.RosPack()

    @staticmethod
    def is_ros_node_alive(node_name: str) -> bool:
        """Return whether ros node is alive or not

        Args:
            node_name (str): ros node name

        Returns:
            bool: True is ros node is alive, False otherwise.
        """
        if node_name in rosnode.get_node_names():
            return True
        return False

    @staticmethod
    def wait_for_rosnode(alive_nodes: Optional[list] = None,
                         dead_nodes: Optional[list] = None,
                         max_retry_attempts: int = 10,
                         backoff_time_sec: float = 1.0) -> None:
        """Wait for starting/killing ros node to complete

        Args:
            alive_nodes(Optional[list]): list of alive nodes which should be started
            dead_nodes(Optional[list]): list of dead nodes which should be killed
            max_retry_attempts (int): max retry attempts for waiting ROS node check to complete
            backoff_time_sec (float): backoff time in seconds for ROS node check to complete

        Raises:
            DeepRacerEnvException: exception if starting/killing ros node fails to complete
        """
        try_count = 0
        alive_nodes = alive_nodes or list()
        dead_nodes = dead_nodes or list()
        while True:
            if all([ROSUtil.is_ros_node_alive(node) for node in alive_nodes]) and \
                    all([not ROSUtil.is_ros_node_alive(node) for node in dead_nodes]):
                break
            try_count += 1
            if try_count > max_retry_attempts:
                raise KvsException(
                    "[ROSUtil]: wait_for_rosnode starting ros node {} "
                    "or killing ros node {} failed".format(alive_nodes, dead_nodes))
            rospy.loginfo("[ROSUtil]: wait_for_rosnode starting ros node {} "
                          "or killing ros node {}".format(alive_nodes, dead_nodes))
            time.sleep(backoff_time_sec)
