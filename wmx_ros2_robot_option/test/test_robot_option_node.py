"""Integration test: wmx_robot_option_node service/topic availability."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger

from wmx_ros2_message.msg import RobotArmStatus
from wmx_ros2_message.srv import RobotCheckAngle
from wmx_ros2_message.srv import RobotCheckPose
from wmx_ros2_message.srv import RobotJogPose
from wmx_ros2_message.srv import RobotMoveAngle
from wmx_ros2_message.srv import RobotMovePose
from wmx_ros2_message.srv import SetRobotScalar


@pytest.mark.launch_test
def generate_test_description():
    engine_node = launch_ros.actions.Node(
        package='wmx_ros2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
    )
    robot_option_node = launch_ros.actions.Node(
        package='wmx_ros2_robot_option',
        executable='wmx_robot_option_node',
        name='wmx_robot_option_node',
        output='screen',
    )

    return launch.LaunchDescription([
        engine_node,
        robot_option_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'engine_node': engine_node, 'robot_option_node': robot_option_node}


class TestRobotOptionNode(unittest.TestCase):
    """Verify wmx_robot_option_node exposes its control surface."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_robot_option_node')

    def tearDown(self):
        self.node.destroy_node()

    def _assert_service(self, srv_type, name):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=20),
            f'{name} service not available',
        )

    def test_set_servo_service(self):
        self._assert_service(SetBool, 'wmx/robot/set_servo')

    def test_set_speed_service(self):
        self._assert_service(SetRobotScalar, 'wmx/robot/set_speed')

    def test_clear_errors_service(self):
        self._assert_service(Trigger, 'wmx/robot/clear_errors')

    def test_stop_motion_service(self):
        self._assert_service(Trigger, 'wmx/robot/stop_motion')

    def test_export_params_service(self):
        self._assert_service(Trigger, 'wmx/robot/export_params')

    def test_jog_pose_service(self):
        self._assert_service(RobotJogPose, 'wmx/robot/jog_pose')

    def test_jog_pose_absolute_service(self):
        self._assert_service(RobotJogPose, 'wmx/robot/jog_pose_absolute')

    def test_set_pose_ptp_service(self):
        self._assert_service(RobotMovePose, 'wmx/robot/set_pose_ptp')

    def test_jog_angle_service(self):
        self._assert_service(RobotMoveAngle, 'wmx/robot/jog_angle')

    def test_jog_angle_absolute_service(self):
        self._assert_service(RobotMoveAngle, 'wmx/robot/jog_angle_absolute')

    def test_set_angle_ptp_service(self):
        self._assert_service(RobotMoveAngle, 'wmx/robot/set_angle_ptp')

    def test_check_pose_service(self):
        self._assert_service(RobotCheckPose, 'wmx/robot/check_pose')

    def test_check_angle_service(self):
        self._assert_service(RobotCheckAngle, 'wmx/robot/check_angle')

    def test_set_collision_enable_service(self):
        self._assert_service(SetBool, 'wmx/robot/set_collision_enable')

    def test_set_fitting_param_service(self):
        self._assert_service(SetBool, 'wmx/robot/set_fitting_param')

    def test_set_collision_sensitivity_service(self):
        self._assert_service(SetRobotScalar, 'wmx/robot/set_collision_sensitivity')

    def test_status_topic_available(self):
        """The robot status topic should be advertised."""
        sub = self.node.create_subscription(
            RobotArmStatus, 'wmx/robot/status', lambda _msg: None, 1)
        self.addCleanup(self.node.destroy_subscription, sub)
        self.assertIsNotNone(sub)


@launch_testing.post_shutdown_test()
class TestRobotOptionNodeShutdown(unittest.TestCase):
    """Verify the option node exits cleanly."""

    def test_exit_code(self, proc_info, robot_option_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15],
            process=robot_option_node,
        )
