#!/usr/bin/python3
import os
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np


class LegJointCommanderClient(Node):
    def __init__(self, namespace=''):
        super().__init__('leg_joint_action_client')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self._state_sub = self.create_subscription(JointState, '/joint_states', self.state_callback, qos_profile)
        self._namespace = namespace
        self._cur_position = np.array([0.0, 0.0])

    def send_goal(self, angle):
        goal_msg = FollowJointTrajectory.Goal()

        joint_name = ["lf_hip_abd",
                        "lf_hip_pitch",
                        "lf_knee_pitch"]
        points = []
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point.positions = [-angle/2, angle, -2*angle]

        points.append(point)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_name
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goals(self, angles):
        goal_msg = FollowJointTrajectory.Goal()

        joint_name = ["lf_hip_abd",
                        "lf_hip_pitch",
                        "lf_knee_pitch"]
        points = []
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point.positions = [angles[0], angles[1], angles[2]]

        points.append(point)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_name
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def inverse_kinematics(self, position):
        import math

        x = float(position[0])
        y = float(position[1])
        z = float(position[2])

    # Link lengths (from the lab model dimensions: thigh 10 cm, shank 10 cm)
        L1 = 0.15  # thigh (m)
        L2 = 0.15  # shank (m)

    # Helper
        def clamp(v, lo=-1.0, hi=1.0):
            return max(lo, min(hi, v))

    # ------------------------------------------------------------
    # Joint order assumed by the simple leg in the lab:
    # [hip_abd, hip_pitch, knee_pitch]
    #
    # hip_abd rotates about the x-axis -> brings (y,z) into the sagittal plane
    # hip_pitch + knee_pitch solve a 2-link planar IK in (x, z_plane)
    # ------------------------------------------------------------

    # 1) Hip abduction/adduction: rotate around x so the leg lies in the x–z plane
    # With z typically negative "down" in the lab command examples.
        hip_abd = math.atan2(y, -z)

    # Distance from hip to foot in the (y,z) plane after undoing hip_abd
        r_yz = math.sqrt(y * y + z * z)

    # Keep the "down" direction consistent for the planar solve
        z_plane = -r_yz

    # 2) Planar 2-link IK for hip_pitch and knee_pitch
        D = math.sqrt(x * x + z_plane * z_plane)

    # Unreachable: clamp to edge of workspace (prevents NaNs)
    # (Optionally you can raise an exception instead.)
        D = max(1e-9, min(D, L1 + L2 - 1e-9))

    # Knee angle (internal angle)
        c_knee = clamp((D * D - L1 * L1 - L2 * L2) / (2.0 * L1 * L2))
        knee_internal = math.acos(c_knee)

    # Convert to joint convention used in many ROS leg models:
    # knee_pitch negative when the knee bends "forward/down".
        knee_pitch = -knee_internal

    # Hip pitch using standard 2-link geometry
        phi = math.atan2(-z_plane, x)  # angle to target in sagittal plane
        psi = math.atan2(L2 * math.sin(knee_internal), L1 + L2 * math.cos(knee_internal))
        hip_pitch = phi - psi

        joint_angles = [hip_abd, hip_pitch, knee_pitch]
        return joint_angles

    
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
        self.get_logger().info('Result: '+str(result))
        raise SystemExit

    def feedback_callback(self, feedback_msg):
        angle_fb = feedback_msg.feedback.actual.positions
        angle_des = feedback_msg.feedback.desired.positions
        self.get_logger().info(f'Desired angles: {angle_des[0]:.2f}, {angle_des[1]:.2f}, {angle_des[2]:.2f}')
        self.get_logger().info(f'Current angles: {angle_fb[0]:.2f}, {angle_fb[1]:.2f}, {angle_fb[2]:.2f}')


    def state_callback(self, state_msg):
        self._cur_position = state_msg.position

    

def main(args=None):
    
    rclpy.init()

    namespace = "robot"
    action_client = LegJointCommanderClient(namespace)
    if len(sys.argv) == 2:
        angle = float(sys.argv[1])
        future = action_client.send_goal(angle)
    elif len(sys.argv) == 4:
        angles = action_client.inverse_kinematics([sys.argv[1], sys.argv[2], sys.argv[3]])
        future = action_client.send_goals(angles)
    try:
        rclpy.spin(action_client)
    except SystemExit:
        rclpy.logging.get_logger('leg_joint_controller_client').info('Shutting down the leg_joint_controller_client')
    action_client.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()