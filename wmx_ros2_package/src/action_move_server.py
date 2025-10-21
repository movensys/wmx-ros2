#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import os

class FollowJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('dobot_group_controller')
        self._action_server = ActionServer(self,FollowJointTrajectory,f'/cr3_group_controller/follow_joint_trajectory',self.execute_callback)
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
                joint.append(180 * ii / 3.14159)
            Positions.append(joint)
            self.get_logger().info(
                "Point {}: Positions: {}, Velocities: {}, Accelerations: {}, TimeFromStart: {}".format(
                    i, joint, point.velocities, point.accelerations, point.time_from_start.sec
                )
            )
        for ii in Positions:
            print("haha")
            #self.ServoJ_C(ii[0],ii[1],ii[2],ii[3],ii[4],ii[5])
            time.sleep(0.18)
    
    #def ServoJ_C(self, j1, j2, j3, j4, j5, j6): 
    #    P1 = ServoJ.Request()
    #    P1.a = float(j1)
    #    P1.b = float(j2)
    #    P1.c = float(j3)
    #    P1.d = float(j4)
    #    P1.e = float(j5)
    #    P1.f = float(j6)
    #    P1.param_value = ["t=0.2"]
    #    response = self.ServoJ_l.call_async(P1)
    

def main(args=None):
    rclpy.init(args=args)
    follow_joint_trajectory_server = FollowJointTrajectoryServer()
    rclpy.spin(follow_joint_trajectory_server)
    follow_joint_trajectory_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()