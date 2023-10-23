#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
"""Script to start DeepRacer NodeMonitoring."""
import logging
import argparse

from importlib import reload
from typing import List, Optional
from node_monitor import NodeMonitor
from deepracer_node_monitor import DeepRacerNodeMonitor


def get_node_monitor_list(node_monitor_file_path: Optional[str] = None) -> List[str]:
    """
    Parse the text file containing all the nodes to be monitored

    Args:
        node_monitor_file_path (Optional[str]): Path to the text file containing ROS nodes to be monitored

    Returns:
        List[str]: list of nodes to be monitored
    """
    node_monitor_list = list()
    if node_monitor_file_path:
        with open(node_monitor_file_path, "r") as fp:
            node_monitor_list = [node for node in fp.read().split('\n') if node]
    return node_monitor_list


def main() -> None:
    """
    Main function for the script
    """
    reload(logging)
    logging.basicConfig(format='%(asctime)s %(levelname)s:%(message)s', level=logging.INFO, datefmt='%I:%M:%S')
    parser = argparse.ArgumentParser()
    parser.add_argument('--node_monitor_file_path',
                        help='(string) path to file containing list of nodes to be monitored',
                        type=str,
                        default='')
    args = parser.parse_args()

    node_monitor_list = get_node_monitor_list(args.node_monitor_file_path)
    deepracer_node_monitor = DeepRacerNodeMonitor(monitor_nodes=node_monitor_list)
    node_monitor = NodeMonitor(monitor_nodes=node_monitor_list,
                               update_rate_hz=1)
    node_monitor.register(deepracer_node_monitor)
    node_monitor.start()


if __name__ == '__main__':
    main()
