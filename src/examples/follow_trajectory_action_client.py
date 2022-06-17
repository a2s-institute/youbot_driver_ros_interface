#!/usr/bin/env python3

import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

from ament_index_python.packages import get_package_share_directory

class FollowJointTrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('follow_joint_trajectory_client', allow_undeclared_parameters = True, automatically_declare_parameters_from_overrides = True)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_1/arm_controller/follow_joint_trajectory')
        
        path = get_package_share_directory('youbot_driver_ros_interface')+'/../../lib/youbot_driver_ros_interface/trajectory.json'

        f = open(path)
        self.data = json.load(f)

        
    def send_goal(self):

        header = Header()

        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'arm_link_0'
    
        goal_msg = FollowJointTrajectory.Goal()
        
        goal_msg.trajectory = JointTrajectory()

        goal_msg.trajectory.header = header
        goal_msg.trajectory.joint_names = self.data.get('goal').get('trajectory').get('joint_names')

        for point in self.data['goal']['trajectory']['points']:
                    trajectory_point = JointTrajectoryPoint(
                        positions=point['positions'],
                        velocities=point['velocities'],
                        accelerations=point['accelerations'],
                        time_from_start=Duration(
                            sec=point['time_from_start']['secs'],
                            nanosec=point['time_from_start']['nsecs']
                        )
                    )
                    goal_msg.trajectory.points.append(trajectory_point)


        self.get_logger().info('waiting for server')
        self._action_client.wait_for_server()
        self.get_logger().info('sending goal')

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FollowJointTrajectoryActionClient()

    future = action_client.send_goal()
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()