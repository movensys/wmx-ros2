#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import os

class FollowJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('dobot_group_controller')
        self._action_server = ActionServer(self,FollowJointTrajectory,f'/cr3_group_controller/follow_joint_trajectory',self.execute_callback)
        
        self.trajectory_publisher = self.create_publisher(Float64MultiArray, '/mvsk/trajectory', 1)

        self.get_logger().info("FollowJointTrajectory Action Server is ready...")
        
    async def execute_callback(self, goal_handle):
        self.get_logger().info("Received a new trajectory goal!")
        trajectory = goal_handle.request.trajectory
        self.execution_trajectory(trajectory)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

    def execution_trajectory(self, trajectory: JointTrajectory):
        self.get_logger().info("Joint Names: {}".format(trajectory.joint_names))
        Positions = []
        for i, point in enumerate(trajectory.points):
            joint= []
            for ii in point.positions:
                joint.append(ii)
            Positions.append(joint)
            self.get_logger().info(
                "Point {}: Positions: {}, Velocities: {}, Accelerations: {}, TimeFromStart: {}".format(
                    i, joint, point.velocities, point.accelerations, point.time_from_start.nanosec
                )
            )

        self.get_logger().info(
                "Command Start!!!"
            )

        for i in range (len(trajectory.points)):
            trajectory_msgs = Float64MultiArray()
            for j in range(6):
                trajectory_msgs.data.append(trajectory.points[i].positions[j]) #position
            for j in range(6):
                trajectory_msgs.data.append(trajectory.points[i].velocities[j]) #velocity
            for j in range(6):
                trajectory_msgs.data.append(trajectory.points[i].accelerations[j]) #acceleration
            for j in range(6):
                if i == 0:
                    trajectory_msgs.data.append(trajectory.points[i].velocities[j]) #prev velocity
                else:
                    trajectory_msgs.data.append(trajectory.points[i-1].velocities[j]) #prev velocity
            
            self.trajectory_publisher.publish(trajectory_msgs)
            
            time.sleep(0.099)

def main(args=None):
    rclpy.init(args=args)
    follow_joint_trajectory_server = FollowJointTrajectoryServer()
    rclpy.spin(follow_joint_trajectory_server)
    follow_joint_trajectory_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()