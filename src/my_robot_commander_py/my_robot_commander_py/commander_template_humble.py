#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray


class ArmCommander(Node):
    def __init__(self):
        super().__init__("arm_commander")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory"
        )

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory"
        )

        self.last_pos = None

        self.sub_cmd = self.create_subscription(
            Float64MultiArray,
            "arm_cmd",
            self.call_back_arm_cmd,
            1
        )

    def call_back_arm_cmd(self, msg: Float64MultiArray):
        data = list(msg.data)

        if len(data) < 7:
            self.get_logger().error(
                f"arm_cmd cần 7 phần tử: "
                f"[gripper, joint1, joint2, joint3, joint4, joint5, joint6], "
                f"nhưng nhận {len(data)}"
            )
            return

        self.last_pos = data

        self.get_logger().info(
            "Received arm_cmd: "
            f"gripper={data[0]:.3f}, "
            f"joint1={data[1]:.3f}, "
            f"joint2={data[2]:.3f}, "
            f"joint3={data[3]:.3f}, "
            f"joint4={data[4]:.3f}, "
            f"joint5={data[5]:.3f}, "
            f"joint6={data[6]:.3f}"
        )

        self.send_arm_goal(data)
        self.send_gripper_goal(data[0])

    def send_arm_goal(self, pos):
        if not self.arm_client.server_is_ready():
            self.get_logger().info("Waiting for arm_controller action server...")
            self.arm_client.wait_for_server(timeout_sec=2.0)

        if not self.arm_client.server_is_ready():
            self.get_logger().error("arm_controller action server not available")
            return

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

        point = JointTrajectoryPoint()

        # data[0] = gripper
        # data[1]~data[6] = joint1~joint6
        point.positions = [
            pos[1],
            pos[2],
            pos[3],
            pos[4],
            pos[5],
            pos[6],
        ]

        point.time_from_start.sec = 3
        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending arm trajectory...")
        future = self.arm_client.send_goal_async(goal_msg)
        future.add_done_callback(self.arm_goal_response_callback)

    def arm_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Arm goal rejected")
            return

        self.get_logger().info("Arm goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Arm trajectory done, status={result.status}")

    def send_gripper_goal(self, gripper_pos):
        if not self.gripper_client.server_is_ready():
            self.get_logger().info("Waiting for gripper_controller action server...")
            self.gripper_client.wait_for_server(timeout_sec=2.0)

        if not self.gripper_client.server_is_ready():
            self.get_logger().error("gripper_controller action server not available")
            return

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = [
            "gripper_left_finger_joint"
        ]

        point = JointTrajectoryPoint()
        point.positions = [
            gripper_pos
        ]

        point.time_from_start.sec = 2
        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending gripper trajectory...")
        future = self.gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return

        self.get_logger().info("Gripper goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Gripper trajectory done, status={result.status}")


def main(args=None):
    rclpy.init(args=args)

    node = ArmCommander()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()