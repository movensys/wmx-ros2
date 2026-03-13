"""Integration test: wmx_io_node service availability and basic calls."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node

from wmx_ros2_message.srv import GetIoBit
from wmx_ros2_message.srv import GetIoBytes
from wmx_ros2_message.srv import SetIoBit
from wmx_ros2_message.srv import SetIoBytes


@pytest.mark.launch_test
def generate_test_description():
    engine_node = launch_ros.actions.Node(
        package='wmx_ros2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
    )
    io_node = launch_ros.actions.Node(
        package='wmx_ros2_package',
        executable='wmx_io_node',
        name='wmx_io_node',
        output='screen',
    )

    return launch.LaunchDescription([
        engine_node,
        io_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'engine_node': engine_node, 'io_node': io_node}


class TestIoNode(unittest.TestCase):
    """Test wmx_io_node service availability and responses."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_io_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_get_input_bit_service(self):
        """get_input_bit service should be available and respond."""
        client = self.node.create_client(GetIoBit, 'wmx/io/get_input_bit')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_input_bit service not available',
        )

        req = GetIoBit.Request()
        req.byte = 0
        req.bit = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_get_output_bit_service(self):
        """get_output_bit service should be available and respond."""
        client = self.node.create_client(GetIoBit, 'wmx/io/get_output_bit')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_output_bit service not available',
        )

        req = GetIoBit.Request()
        req.byte = 0
        req.bit = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_get_input_bytes_service(self):
        """get_input_bytes service should be available and respond."""
        client = self.node.create_client(GetIoBytes, 'wmx/io/get_input_bytes')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_input_bytes service not available',
        )

        req = GetIoBytes.Request()
        req.byte = 0
        req.length = 1
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_get_output_bytes_service(self):
        """get_output_bytes service should be available and respond."""
        client = self.node.create_client(GetIoBytes, 'wmx/io/get_output_bytes')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/get_output_bytes service not available',
        )

        req = GetIoBytes.Request()
        req.byte = 0
        req.length = 1
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')

    def test_set_output_bit_service(self):
        """set_output_bit service should be available."""
        client = self.node.create_client(SetIoBit, 'wmx/io/set_output_bit')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/set_output_bit service not available',
        )

    def test_set_output_bytes_service(self):
        """set_output_bytes service should be available."""
        client = self.node.create_client(SetIoBytes, 'wmx/io/set_output_bytes')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/io/set_output_bytes service not available',
        )


@launch_testing.post_shutdown_test()
class TestIoNodeShutdown(unittest.TestCase):
    """Verify io node exits cleanly."""

    def test_exit_code(self, proc_info, io_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15],
            process=io_node,
        )
