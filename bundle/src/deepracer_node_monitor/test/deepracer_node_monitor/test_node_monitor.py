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
from unittest import TestCase
from unittest.mock import patch, MagicMock

# rclpy is imported in NodeMonitor class, but this package is not a ROS environment.
# Since this is missing the rclpy, mocking the rclpy module.
# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
from deepracer_node_monitor.node_monitor import NodeMonitor

sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.parameter_service"] = MagicMock()
sys.modules["rcl_interfaces.srv"] = MagicMock()
sys.modules["markov.log_handler.constants"] = MagicMock()
sys.modules["markov.log_handler.exception_handler"] = MagicMock()
sys.modules["markov.utils"] = MagicMock()
sys.modules["markov.constants"] = MagicMock()


@patch("deepracer_node_monitor.node_monitor.RLock")
class NodeMonitorTest(TestCase):
    def setUp(self) -> None:
        self.monitor_nodes = [
            "/gazebo",
            "/deepracer",
            "/ude_ros_server",
            "*/controller_manager",
            "/agent0/robot_state_publisher",
        ]
        self.update_rate_hz = 0.2

    def test_initialize(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        self.assertEqual(node_monitor.monitor_nodes, self.monitor_nodes)
        self.assertEqual(node_monitor.update_rate_hz, self.update_rate_hz)
        self.assertEqual(node_monitor.running_nodes, set())
        self.assertEqual(node_monitor.dead_nodes, set())
        self.assertEqual(rlock_mock.call_count, 4)
        self.assertEqual(node_monitor._observers, set())
        self.assertFalse(node_monitor._is_monitoring)
        self.assertEqual(node_monitor._seen_nodes, set())

    def test_register(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1 = MagicMock()
        observer2 = MagicMock()
        node_monitor.register(observer1)
        node_monitor.register(observer2)
        self.assertEqual(node_monitor._observers, set({observer1, observer2}))

    def test_unregister(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1 = MagicMock()
        observer2 = MagicMock()
        node_monitor._observer_lock = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor.unregister(observer1)
        self.assertEqual(node_monitor._observers, set({observer2}))

    def test_is_monitor_node_true(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        self.assertTrue(node_monitor._is_monitor_node("/gazebo"))
        self.assertTrue(node_monitor._is_monitor_node("/agent0/controller_manager"))
        self.assertTrue(node_monitor._is_monitor_node("/abc/controller_manager"))

    def test_is_monitor_node_false(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        self.assertFalse(node_monitor._is_monitor_node("/hello"))
        self.assertFalse(node_monitor._is_monitor_node("/controller"))

    def test_update_running_nodes_with_newly_updated_running_node(
        self, rlock_mock
    ):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._is_monitoring = True
        # Mock the ROS2 node and its methods
        node_monitor._ros_node = MagicMock()
        node_monitor._ros_node.get_node_names_and_namespaces.return_value = [("gazebo", "/")]
        node_monitor._ros_node.create_client = MagicMock()
        
        # Mock client behavior
        client_mock = MagicMock()
        client_mock.wait_for_service.return_value = True
        future_mock = MagicMock()
        future_mock.done.return_value = True
        future_mock.result.return_value = MagicMock()
        client_mock.call_async.return_value = future_mock
        node_monitor._ros_node.create_client.return_value = client_mock
        
        node_monitor._update_running_nodes()
        
        # Verify create_client was called
        node_monitor._ros_node.create_client.assert_called()
        self.assertEqual(node_monitor._seen_nodes, set({"/gazebo"}))
        self.assertEqual(node_monitor.running_nodes, set({"/gazebo"}))

    def test_update_running_nodes_with_newly_updated_running_node_empty_monitor_node(
        self, rlock_mock
    ):
        node_monitor = NodeMonitor(list(), self.update_rate_hz)
        node_monitor._is_monitoring = True
        # Mock the ROS2 node and its methods
        node_monitor._ros_node = MagicMock()
        node_monitor._ros_node.get_node_names_and_namespaces.return_value = [("gazebo", "/")]
        node_monitor._ros_node.create_client = MagicMock()
        
        # Mock client behavior
        client_mock = MagicMock()
        client_mock.wait_for_service.return_value = True
        future_mock = MagicMock()
        future_mock.done.return_value = True
        future_mock.result.return_value = MagicMock()
        client_mock.call_async.return_value = future_mock
        node_monitor._ros_node.create_client.return_value = client_mock
        
        node_monitor._update_running_nodes()
        
        # Verify create_client was called
        node_monitor._ros_node.create_client.assert_called()
        self.assertEqual(node_monitor._seen_nodes, set({"/gazebo"}))
        self.assertEqual(node_monitor.running_nodes, set({"/gazebo"}))

    def test_update_running_nodes_with_newly_updated_not_in_monitor(
        self, rlock_mock
    ):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._is_monitoring = True
        # Mock the ROS2 node and its methods
        node_monitor._ros_node = MagicMock()
        node_monitor._ros_node.get_node_names_and_namespaces.return_value = [("not_in_monitor", "/")]
        node_monitor._ros_node.create_client = MagicMock()
        
        # Mock client behavior
        client_mock = MagicMock()
        client_mock.wait_for_service.return_value = True
        future_mock = MagicMock()
        future_mock.done.return_value = True
        future_mock.result.return_value = MagicMock()
        client_mock.call_async.return_value = future_mock
        node_monitor._ros_node.create_client.return_value = client_mock
        
        node_monitor._update_running_nodes()
        
        # Verify create_client was called
        node_monitor._ros_node.create_client.assert_called()
        self.assertEqual(node_monitor._seen_nodes, set())
        self.assertEqual(node_monitor.running_nodes, set())

    def test_update_running_nodes_already_seen_node(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._is_monitoring = True
        # Mock the ROS2 node and its methods
        node_monitor._ros_node = MagicMock()
        node_monitor._ros_node.get_node_names_and_namespaces.return_value = [("gazebo", "/")]
        node_monitor._ros_node.create_client = MagicMock()
        
        # Mock client behavior
        client_mock = MagicMock()
        client_mock.wait_for_service.return_value = True
        future_mock = MagicMock()
        future_mock.done.return_value = True
        future_mock.result.return_value = MagicMock()
        client_mock.call_async.return_value = future_mock
        node_monitor._ros_node.create_client.return_value = client_mock
        
        node_monitor._seen_nodes = set({"/gazebo"})
        node_monitor._running_nodes = set({"/gazebo"})
        node_monitor._update_running_nodes()
        
        # Verify create_client was called
        node_monitor._ros_node.create_client.assert_called()

    def test_get_service_client_creates_new(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._ros_node = MagicMock()
        client_mock = MagicMock()
        node_monitor._ros_node.create_client.return_value = client_mock

        result = node_monitor._get_service_client("/gazebo/list_parameters")

        node_monitor._ros_node.create_client.assert_called_once()
        self.assertEqual(result, client_mock)
        self.assertIn("/gazebo/list_parameters", node_monitor._service_clients)

    def test_get_service_client_returns_cached(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._ros_node = MagicMock()
        cached_client = MagicMock()
        node_monitor._service_clients["/gazebo/list_parameters"] = cached_client

        result = node_monitor._get_service_client("/gazebo/list_parameters")

        node_monitor._ros_node.create_client.assert_not_called()
        self.assertEqual(result, cached_client)

    def test_destroy_service_client(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._ros_node = MagicMock()
        client_mock = MagicMock()
        node_monitor._service_clients["/gazebo/list_parameters"] = client_mock

        node_monitor._destroy_service_client("/gazebo/list_parameters")

        node_monitor._ros_node.destroy_client.assert_called_once_with(client_mock)
        self.assertNotIn("/gazebo/list_parameters", node_monitor._service_clients)

    def test_destroy_service_client_not_present(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._ros_node = MagicMock()

        node_monitor._destroy_service_client("/nonexistent/list_parameters")

        node_monitor._ros_node.destroy_client.assert_not_called()

    def test_destroy_service_client_exception(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._ros_node = MagicMock()
        client_mock = MagicMock()
        node_monitor._service_clients["/gazebo/list_parameters"] = client_mock
        node_monitor._ros_node.destroy_client.side_effect = Exception("destroy failed")

        node_monitor._destroy_service_client("/gazebo/list_parameters")

        self.assertNotIn("/gazebo/list_parameters", node_monitor._service_clients)

    def test_prune_stale_service_clients(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._ros_node = MagicMock()
        active_client = MagicMock()
        stale_client = MagicMock()
        node_monitor._service_clients["/gazebo/list_parameters"] = active_client
        node_monitor._service_clients["/old_node/list_parameters"] = stale_client

        node_monitor._prune_stale_service_clients({"/gazebo/list_parameters"})

        self.assertIn("/gazebo/list_parameters", node_monitor._service_clients)
        self.assertNotIn("/old_node/list_parameters", node_monitor._service_clients)
        node_monitor._ros_node.destroy_client.assert_called_once_with(stale_client)

    def test_destroy_all_service_clients(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._ros_node = MagicMock()
        node_monitor._service_clients["/gazebo/list_parameters"] = MagicMock()
        node_monitor._service_clients["/deepracer/list_parameters"] = MagicMock()

        node_monitor._destroy_all_service_clients()

        self.assertEqual(node_monitor._ros_node.destroy_client.call_count, 2)
        self.assertEqual(len(node_monitor._service_clients), 0)

    @patch("deepracer_node_monitor.node_monitor.logging")
    def test_destroy_all_service_clients_with_exception(self, logging_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._ros_node = MagicMock()
        node_monitor._service_clients["/gazebo/list_parameters"] = MagicMock()
        node_monitor._service_clients["/deepracer/list_parameters"] = MagicMock()
        node_monitor._ros_node.destroy_client.side_effect = [Exception("fail"), None]

        node_monitor._destroy_all_service_clients()

        self.assertEqual(node_monitor._ros_node.destroy_client.call_count, 2)
        self.assertEqual(len(node_monitor._service_clients), 0)
        logging_mock.exception.assert_called_once()
        self.assertIn("/gazebo/list_parameters", logging_mock.exception.call_args[0][0])

    def test_ros2_ping_all_returns_empty_when_not_monitoring(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._is_monitoring = False

        result = node_monitor._ros2_ping_all()

        self.assertEqual(result, [])

    def test_ros2_ping_all_returns_empty_on_node_discovery_exception(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._is_monitoring = True
        node_monitor._ros_node = MagicMock()
        node_monitor._ros_node.get_node_names_and_namespaces.side_effect = Exception("discovery failed")

        result = node_monitor._ros2_ping_all()

        self.assertEqual(result, [])

    def test_ros2_ping_all_destroys_client_on_unexpected_exception(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._is_monitoring = True
        node_monitor._ros_node = MagicMock()
        node_monitor._ros_node.get_node_names_and_namespaces.return_value = [("gazebo", "/")]
        client_mock = MagicMock()
        client_mock.wait_for_service.side_effect = Exception("unexpected")
        node_monitor._ros_node.create_client.return_value = client_mock

        result = node_monitor._ros2_ping_all()

        self.assertEqual(result, [])
        node_monitor._ros_node.destroy_client.assert_called_once_with(client_mock)

    def test_update_running_nodes_failed_master_node(self, rlock_mock):
        with self.assertRaises(Exception):
            node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
            node_monitor._update_running_nodes.side_effect = Exception()
            node_monitor._update_running_nodes()
            self.assertEqual(node_monitor._seen_nodes, set())
            self.assertEqual(node_monitor.running_nodes, set())

    def test_update_dead_nodes_success(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._seen_nodes = set({"/gazebo", "/deepracer"})
        node_monitor._running_nodes = set()
        node_monitor._dead_nodes = set()
        node_monitor._update_dead_nodes()
        self.assertEqual(node_monitor._dead_nodes, set({"/gazebo", "/deepracer"}))

    def test_update_dead_nodes_already_in_list(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        node_monitor._seen_nodes = set({"/gazebo"})
        node_monitor._running_nodes = set()
        node_monitor._dead_nodes = set({"/gazebo"})
        node_monitor._update_dead_nodes()
        self.assertEqual(node_monitor._dead_nodes, set({"/gazebo"}))

    @patch("time.sleep", side_effect=InterruptedError)
    def test_start(self, sleep_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1, observer2 = MagicMock(), MagicMock()
        observer1.on_start = MagicMock()
        observer2.on_start = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor._update_running_nodes = MagicMock()
        node_monitor._update_dead_nodes = MagicMock()
        node_monitor.checkIfROSNodeMonitorShouldStop = MagicMock()
        node_monitor.checkIfROSNodeMonitorShouldStop.return_value = False
        node_monitor.stopSimulationJob = MagicMock()
        node_monitor.start()
        observer1.on_start.assert_called_once()
        observer2.on_start.assert_called_once()
        node_monitor._update_running_nodes.assert_called_once()
        node_monitor._update_dead_nodes.assert_called_once()
        self.assertTrue(node_monitor._is_monitoring)

    @patch("time.sleep", side_effect=InterruptedError)
    def test_start_calling_running_dead(self, sleep_mock, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1, observer2 = MagicMock(), MagicMock()
        observer1.on_start = MagicMock()
        observer2.on_start = MagicMock()
        observer1.on_running_node_update = MagicMock()
        observer2.on_running_node_update = MagicMock()
        observer1.on_dead_node_update = MagicMock()
        observer2.on_dead_node_update = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor._update_running_nodes = MagicMock()
        node_monitor._update_dead_nodes = MagicMock()
        node_monitor._running_nodes = MagicMock()
        node_monitor._dead_nodes = MagicMock()
        node_monitor._running_nodes.copy.return_value = set({"/gazebo"})
        node_monitor._dead_nodes.copy.return_value = set({"/gazebo"})
        node_monitor.checkIfROSNodeMonitorShouldStop = MagicMock()
        node_monitor.checkIfROSNodeMonitorShouldStop.return_value = False
        node_monitor.stopSimulationJob = MagicMock()
        node_monitor.start()
        observer1.on_start.assert_called_once()
        observer2.on_start.assert_called_once()
        node_monitor._update_running_nodes.assert_called_once()
        node_monitor._update_dead_nodes.assert_called_once()
        observer1.on_running_node_update.assert_called_once()
        observer2.on_running_node_update.assert_called_once()
        observer1.on_dead_node_update.assert_called_once()
        observer2.on_dead_node_update.assert_called_once()
        self.assertTrue(node_monitor._is_monitoring)

    def test_stop(self, rlock_mock):
        node_monitor = NodeMonitor(self.monitor_nodes, self.update_rate_hz)
        observer1, observer2 = MagicMock(), MagicMock()
        observer1.on_stop = MagicMock()
        observer2.on_stop = MagicMock()
        node_monitor._observers = set({observer1, observer2})
        node_monitor.stop()
        observer1.on_stop.assert_called_once()
        observer2.on_stop.assert_called_once()
        self.assertFalse(node_monitor._is_monitoring)
