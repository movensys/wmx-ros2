"""Integration test: wmx_engine_node lifecycle and ready signal."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    engine_node = launch_ros.actions.Node(
        package='wmx_ros2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
    )

    return launch.LaunchDescription([
        engine_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'engine_node': engine_node}


class TestEngineLifecycle(unittest.TestCase):
    """Test wmx_engine_node startup, ready signal, and services."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_engine_lifecycle')

    def tearDown(self):
        self.node.destroy_node()

    def test_engine_publishes_ready(self):
        """Engine node should publish True on wmx/engine/ready after startup."""
        received = []

        def cb(msg):
            received.append(msg.data)

        self.node.create_subscription(
            Bool,
            'wmx/engine/ready',
            cb,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1,
            ),
        )

        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=15)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            if any(v is True for v in received):
                break

        self.assertTrue(
            any(v is True for v in received),
            'Engine did not publish ready=True within 15 seconds',
        )

    def test_get_engine_status_service(self):
        """wmx/engine/get_status service should return a valid engine state."""
        client = self.node.create_client(Trigger, 'wmx/engine/get_status')

        self.assertTrue(
            client.wait_for_service(timeout_sec=15),
            'get_status service not available within 15 seconds',
        )

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)

        self.assertIsNotNone(future.result(), 'Service call returned no result')
        result = future.result()
        self.assertTrue(result.success)
        valid_states = ['Idle', 'Running', 'Communicating', 'Shutdown', 'Unknown']
        self.assertIn(
            result.message,
            valid_states,
            f'Unexpected engine state: {result.message}',
        )


@launch_testing.post_shutdown_test()
class TestEngineShutdown(unittest.TestCase):
    """Verify engine node exits cleanly."""

    def test_exit_code(self, proc_info, engine_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],  # SIGINT, SIGTERM are normal
            process=engine_node,
        )
