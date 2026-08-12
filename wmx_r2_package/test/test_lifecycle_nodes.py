"""Integration test: managed lifecycle nodes and wmx_engine_node state control."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from lifecycle_msgs.srv import GetState
import pytest
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from wmx_r2_message.srv import SetNodeState

LIFECYCLE_NODES = ['wmx_core_motion_node', 'wmx_io_node', 'wmx_ethercat_node']


@pytest.mark.launch_test
def generate_test_description():
    engine_node = launch_ros.actions.Node(
        package='wmx_r2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
    )
    managed = [
        launch_ros.actions.LifecycleNode(
            package='wmx_r2_package',
            executable=name,
            name=name,
            namespace='',
            output='screen',
        )
        for name in LIFECYCLE_NODES
    ]

    return launch.LaunchDescription([
        engine_node,
        *managed,
        launch_testing.actions.ReadyToTest(),
    ]), {'engine_node': engine_node}


class TestLifecycleNodes(unittest.TestCase):
    """Test that the managed nodes are lifecycle nodes driven by the engine."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_lifecycle_nodes')

    def tearDown(self):
        self.node.destroy_node()

    def call(self, client, request, timeout_sec=30):
        """Call a service and return its response, or None on timeout."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        return future.result()

    def test_nodes_expose_lifecycle_services(self):
        """Each node under test should offer the standard lifecycle services."""
        for name in LIFECYCLE_NODES:
            client = self.node.create_client(GetState, f'{name}/get_state')
            self.assertTrue(
                client.wait_for_service(timeout_sec=20),
                f'{name}/get_state service not available',
            )

            result = self.call(client, GetState.Request())
            self.assertIsNotNone(result, f'{name}/get_state returned no result')
            self.assertIn(
                result.current_state.label,
                ['unconfigured', 'inactive', 'active', 'finalized'],
                f'{name} reported an unexpected state',
            )

    def test_engine_reports_node_states(self):
        """wmx/engine/get_node_states should list the lifecycle nodes it finds."""
        client = self.node.create_client(Trigger, 'wmx/engine/get_node_states')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/engine/get_node_states service not available',
        )

        result = self.call(client, Trigger.Request())
        self.assertIsNotNone(result, 'Service call returned no result')
        self.assertTrue(result.success)
        for name in LIFECYCLE_NODES:
            self.assertIn(name, result.message)

    def test_engine_rejects_unknown_transition(self):
        """An unknown transition name should fail instead of doing something."""
        client = self.node.create_client(SetNodeState, 'wmx/engine/set_node_state')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/engine/set_node_state service not available',
        )

        req = SetNodeState.Request()
        req.node_name = 'wmx_io_node'
        req.transition = 'not_a_transition'
        result = self.call(client, req)

        self.assertIsNotNone(result, 'Service call returned no result')
        self.assertFalse(result.success)

    def test_engine_rejects_missing_node_name(self):
        """An empty node_name should be rejected: transitions are per node."""
        client = self.node.create_client(SetNodeState, 'wmx/engine/set_node_state')
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            'wmx/engine/set_node_state service not available',
        )

        req = SetNodeState.Request()
        req.node_name = ''
        req.transition = 'bringup'
        result = self.call(client, req)

        self.assertIsNotNone(result, 'Service call returned no result')
        self.assertFalse(result.success)

    def test_engine_can_deactivate_and_reactivate_io_node(self):
        """The engine should be able to drive a managed node's state."""
        state_client = self.node.create_client(GetState, 'wmx_io_node/get_state')
        self.assertTrue(
            state_client.wait_for_service(timeout_sec=20),
            'wmx_io_node/get_state service not available',
        )

        set_client = self.node.create_client(SetNodeState, 'wmx/engine/set_node_state')
        self.assertTrue(
            set_client.wait_for_service(timeout_sec=20),
            'wmx/engine/set_node_state service not available',
        )

        # The engine discovers the node and brings it up on its own once it is
        # communicating. Without a running engine there is nothing to deactivate.
        state = self.call(state_client, GetState.Request())
        self.assertIsNotNone(state, 'wmx_io_node/get_state returned no result')
        if state.current_state.label != 'active':
            self.skipTest(
                'wmx_io_node is not active (engine unavailable): '
                f'state is {state.current_state.label}')

        req = SetNodeState.Request()
        req.node_name = 'wmx_io_node'
        req.transition = 'deactivate'
        result = self.call(set_client, req)
        self.assertIsNotNone(result, 'deactivate returned no result')
        self.assertTrue(result.success, result.message)
        self.assertEqual(list(result.states), ['inactive'])

        req.transition = 'activate'
        result = self.call(set_client, req)
        self.assertIsNotNone(result, 'activate returned no result')
        self.assertTrue(result.success, result.message)
        self.assertEqual(list(result.states), ['active'])


@launch_testing.post_shutdown_test()
class TestLifecycleNodesShutdown(unittest.TestCase):
    """Verify the engine node exits cleanly."""

    def test_exit_code(self, proc_info, engine_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15],
            process=engine_node,
        )
