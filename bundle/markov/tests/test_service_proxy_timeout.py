# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for the opt-in response timeout of ServiceProxyWrapper (response_timeout_sec).

The services are served by a separate node and executor, as Gazebo serves them in the SimApp, so
the wrapper's own ROS2NodeManager executor only delivers responses.
"""

import threading
import time
import unittest
from unittest import mock

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from markov.rclpy_wrappers import ROS2NodeManager, ServiceProxyWrapper


class _LogAndExitCalled(Exception):
    """Raised by the patched log_and_exit so a test can observe the give-up path."""


class TestServiceProxyResponseTimeout(unittest.TestCase):
    """ServiceProxyWrapper with response_timeout_sec set."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
            # An earlier test (test_ros2_migration.py) may have shut rclpy down, which leaves the
            # cached ROS2NodeManager holding a node from the old context; start a fresh one.
            ROS2NodeManager._instance_map.clear()
        cls.server_node = Node('test_service_proxy_timeout_server')
        cls.slow_calls = 0
        cls.slow_calls_lock = threading.Lock()
        cls.server_node.create_service(Empty, '/test_timeout/fast', lambda req, resp: resp)
        cls.server_node.create_service(Empty, '/test_timeout/slow_first', cls._slow_first)
        cls.executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.server_node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.server_node.destroy_node()

    @classmethod
    def _slow_first(cls, request, response):
        """Answer the first request after 1.5 s and every later request at once."""
        with cls.slow_calls_lock:
            cls.slow_calls += 1
            first = cls.slow_calls == 1
        if first:
            time.sleep(1.5)
        return response

    def test_default_waits_and_returns_response(self):
        """Without response_timeout_sec the call returns the response."""
        proxy = ServiceProxyWrapper('/test_timeout/fast', Empty, wait_for_service=True)
        self.assertIsInstance(proxy(Empty.Request()), Empty.Response)

    def test_timeout_drops_request_and_retry_succeeds(self):
        """A slow first response times out, is dropped, and the retried request succeeds."""
        proxy = ServiceProxyWrapper('/test_timeout/slow_first', Empty, wait_for_service=True,
                                    max_retry_attempts=5, response_timeout_sec=0.3)
        with mock.patch.object(proxy.client, 'remove_pending_request',
                               wraps=proxy.client.remove_pending_request) as remove_pending:
            response = proxy(Empty.Request())
        self.assertIsInstance(response, Empty.Response)
        # rclpy also calls remove_pending_request when a response completes a future, so check that
        # a request was removed while still unanswered.
        self.assertTrue(any(not call.args[0].done() for call in remove_pending.call_args_list))
        self.assertGreaterEqual(self.slow_calls, 2)

    def test_gives_up_after_max_retries_without_server(self):
        """With no server, every attempt times out and the wrapper gives up via log_and_exit."""
        proxy = ServiceProxyWrapper('/test_timeout/no_server', Empty,
                                    max_retry_attempts=1, response_timeout_sec=0.2)
        with mock.patch('markov.rclpy_wrappers.log_and_exit', side_effect=_LogAndExitCalled), \
                mock.patch('markov.rclpy_wrappers.ROBOMAKER_CANCEL_JOB_WAIT_TIME', 0), \
                mock.patch.object(proxy.client, 'remove_pending_request',
                                  wraps=proxy.client.remove_pending_request) as remove_pending:
            start = time.monotonic()
            with self.assertRaises(_LogAndExitCalled):
                proxy(Empty.Request())
            elapsed = time.monotonic() - start
        # Two attempts of 0.2 s and one 1 s pause between them; the call must not block longer.
        self.assertLess(elapsed, 5.0)
        # Each timed-out request (the first attempt and its one retry) is removed from the client,
        # so a late response is discarded.
        self.assertEqual(remove_pending.call_count, 2)


if __name__ == '__main__':
    unittest.main()
