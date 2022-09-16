#!/usr/bin/env python3

import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

from ament_index_python.packages import get_package_share_directory

import time

class FollowJointTrajectoryActionClient(Node):

    def __init__(self):
        super().__init__('follow_joint_trajectory_client', allow_undeclared_parameters = True, automatically_declare_parameters_from_overrides = True)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_1/arm_controller/follow_joint_trajectory')

        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.current_joint_state = JointState()

        self.arm_joints = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
        
        path = get_package_share_directory('youbot_driver_ros_interface')+'/../../lib/youbot_driver_ros_interface/trajectory.json'

        f = open(path)
        self.data = json.load(f)

    def joint_states_callback(self, msg):
        self.current_joint_state = msg
        if 'arm' in msg.name[0]:
            # check if positions are  close to zero
            if all(abs(x) < 0.01 for x in msg.position):
                self.send_goal()
                self.destroy_subscription(self.sub_joint_states)
            else:
                self.get_logger().info('Robot is not in proper folded position, please move it to the folded position and restart the node')
                rclpy.shutdown()
        
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

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = FollowJointTrajectoryActionClient()

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()