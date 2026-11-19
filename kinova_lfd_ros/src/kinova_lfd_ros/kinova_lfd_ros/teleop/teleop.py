"""Keyboard controller for SE(2) control (standalone, no Omni/Carb dependency)."""

from __future__ import annotations

from enum import Enum

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, Twist
from kinova_lfd_ros.core.math import base_to_tcp_twist, euler_xyz_to_rotvec
from kinova_lfd_ros.teleop.devices import Se3Keyboard, XboxController
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TeleopState(Enum):
    START = 0
    TELEOP = 1
    SWITCHING_TO_JTC = 2
    HOMING = 3
    SWITCHING_TO_TWIST = 4


class TeleOp(Node):
    """A Teloperation device for recording data."""

    def __init__(self):
        super().__init__("lfd_teleop")

        self.declare_parameter("pos_sensitivity", 0.1)
        self.declare_parameter("rot_sensitivity", 15)
        self.declare_parameter("gripper_term", True)
        self.declare_parameter("reference_frame", "base")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("cmd_topic", "/twist_controller/commands")
        self.declare_parameter("tcp_pose_topic", "/tcp_pose_b")
        self.declare_parameter("listener", "xbox")

        self._pos_sensitivity = self.get_parameter("pos_sensitivity").value
        self._rot_sensitivity = self.get_parameter("rot_sensitivity").value
        self._gripper_term = self.get_parameter("gripper_term").value
        self._reference_frame = self.get_parameter("reference_frame").value
        self._rate = self.get_parameter("publish_rate").value
        self._cmd_topic = self.get_parameter("cmd_topic").value
        self._tcp_pose_topic = self.get_parameter("tcp_pose_topic").value
        self._listener_type = self.get_parameter("listener").value
        self._curr_gripper_goal = 0.0
        self._tcp_pose_b = None

        self._timer = self.create_timer(1.0 / self._rate, self._on_timer)
        self._twist_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._tcp_pose_sub = self.create_subscription(PoseStamped, self._tcp_pose_topic, self._tcp_pose_callback, 10)
        self.gripper_action_client = ActionClient(
            self, ParallelGripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )
        self._state = TeleopState.START
        self._listener = XboxController() if self._listener_type == "xbox" else Se3Keyboard()

        self.get_logger().info(self._listener.__str__())

        # For joint trajectory action
        self.declare_parameter(
            "joint_names",
            [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_7",
                "robotiq_85_left_knuckle_joint",
            ],
        )

        self._joint_names = self.get_parameter("joint_names").value
        self._home_flag = False
        self._prev_home_flag = False
        self._goal_handle = None
        self._goal_active = False
        self._home_joint_positions = np.asarray([0.00, -0.47, 3.14, -2.30, 0.00, -1.25, 1.57, 0.00])

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        self.gripper_action_client = ActionClient(
            self, ParallelGripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

        self._switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

    def advance(self, tcp_pose_b: np.ndarray = None) -> np.ndarray:
        """Return the current command as a tensor.

        Args:
            tcp_pose_b: TCP pose of the robot in the robot base frame.

        Returns:
            np.ndarray: [x, y, z, rx, ry, rz] or [x, y, z, rx, ry, rz, gripper]

        """
        if tcp_pose_b is None:
            return np.zeros(7)
        delta_rot = self._listener._delta_rot * self._rot_sensitivity
        delta_pos = self._listener._delta_pos * self._pos_sensitivity
        gripper_value = self._listener._gripper_value

        if self._reference_frame == "tcp":
            command = np.concatenate((delta_pos, euler_xyz_to_rotvec(delta_rot)), axis=0)
        elif self._reference_frame == "base":
            tcp_lin_vel, tcp_ang_vel = base_to_tcp_twist(delta_pos, delta_rot, tcp_pose_b[3:7])
            command = np.concatenate((tcp_lin_vel, tcp_ang_vel), axis=0)

        if self._gripper_term:
            command = np.concatenate((command, np.asarray([gripper_value])), axis=0)
        return command

    def _tcp_pose_callback(self, pose_msg: PoseStamped) -> None:
        self._tcp_pose_b = np.asarray(
            [
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ]
        )

    def _on_timer(self) -> None:
        self._home_flag = self._listener._home_flag

        if self._state == TeleopState.START:
            future = self._switch_controllers(active=["twist_controller"], inactive=["joint_trajectory_controller"])

        if self._state == TeleopState.HOMING:
            if not self._home_flag:
                self._state = TeleopState.SWITCHING_TO_TWIST
                self._cancel_home_goal()
                return

        if self._state == TeleopState.TELEOP:
            if self._home_flag:
                self._state = TeleopState.SWITCHING_TO_JTC
                future = self._switch_controllers(active=["joint_trajectory_controller"], inactive=["twist_controller"])
                return

            cmd = self.advance(self._tcp_pose_b)

            msg = Twist()
            msg.linear.x = float(cmd[0])
            msg.linear.y = float(cmd[1])
            msg.linear.z = float(cmd[2])
            msg.angular.x = float(cmd[3])
            msg.angular.y = float(cmd[4])
            msg.angular.z = float(cmd[5])
            self._twist_pub.publish(msg)

            gripper_val = max(0, min(cmd[-1], 1)) * 0.8
            if gripper_val != self._curr_gripper_goal:
                self._curr_gripper_goal = gripper_val

                gripper_goal = ParallelGripperCommand.Goal()
                gripper_goal.command.name = ["robotiq_85_left_knuckle_joint"]
                gripper_goal.command.position = [self._curr_gripper_goal]
                gripper_goal.command.effort = [100.0]
                self.gripper_action_client.send_goal_async(gripper_goal)

    def destroy_node(self) -> None:
        super().destroy_node()

    def _check_home_flag(self):
        if self._home_flag and not self._prev_home_flag:
            self._switch_controllers(active=["joint_trajectory_controller"], inactive=["twist_controller"])
            self._send_home_goal()
        elif not self._home_flag and self._prev_home_flag:
            self._cancel_home_goal()
            self._switch_controllers(active=["twist_controller"], inactive=["joint_trajectory_controller"])

        self._prev_home_flag = self._home_flag

    def _build_home_trajectory_goal(self) -> FollowJointTrajectory.Goal:

        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self._joint_names[:-1]

        point = JointTrajectoryPoint()
        point.positions = self._home_joint_positions[:-1]
        point.time_from_start = Duration(sec=5, nanosec=0)

        traj.points = [point]
        goal_msg.trajectory = traj
        return goal_msg

    def _send_home_goal(self):
        if self._goal_active:
            return

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            return

        goal_msg = self._build_home_trajectory_goal()
        self._goal_active = True

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._home_feedback_callback)
        send_goal_future.add_done_callback(self._home_goal_response_callback)

    def _home_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_active = False
            self._goal_handle = None
            return

        self._goal_handle = goal_handle

        if not self._home_flag:
            self._cancel_home_goal()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._home_result_callback)

    def _home_feedback_callback(self, feedback_msg):
        pass

    def _home_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self._goal_active = False
        self._goal_handle = None
        self._state = TeleopState.SWITCHING_TO_TWIST
        self._switch_controllers(active=["twist_controller"], inactive=["joint_trajectory_controller"])

    def _cancel_home_goal(self):
        if self._goal_handle is None:
            return

        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        cancel_response = future.result()
        self._state = TeleopState.SWITCHING_TO_TWIST
        future_switch = self._switch_controllers(active=["twist_controller"], inactive=["joint_trajectory_controller"])
        return future_switch

    def _switch_controllers(
        self, active: list = ["twist_controller"], inactive: list = ["joint_trajectory_controller"]
    ):
        req = SwitchController.Request()

        req.activate_controllers = active
        req.deactivate_controllers = inactive

        req.strictness = 2

        req.activate_asap = True
        req.timeout.sec = 5

        future = self._switch_client.call_async(req)
        future.add_done_callback(self._switch_done)
        return future

    def _switch_done(self, future):

        if self._state == TeleopState.SWITCHING_TO_JTC:
            self._send_home_goal()
            self._state = TeleopState.HOMING

        elif self._state == TeleopState.SWITCHING_TO_TWIST or self._state == TeleopState.START:
            self._state = TeleopState.TELEOP


def main(args=None):
    rclpy.init(args=args)
    node = TeleOp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
