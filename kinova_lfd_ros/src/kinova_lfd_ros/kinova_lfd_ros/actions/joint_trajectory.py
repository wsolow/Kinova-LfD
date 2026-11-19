import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool


class GatedTrajectoryClient(Node):
    """FollowJointTrajectory action client gated by an in-process boolean flag.

    - When self._flag goes False -> True: a trajectory goal is sent.
    - When self._flag goes True -> False while a goal is executing: the goal is
      cancelled immediately, even if the robot hasn't reached the target
      joint positions yet.

    Notes:
    - "Cancel" sends a cancel request to the action server. Most
      joint_trajectory_controller implementations will stop (or decelerate
      and stop) the joints at whatever position they're at when the cancel
      is processed -- it does not snap back or continue to the goal.
    - Set self._flag directly, or call node.set_flag(True/False) from anywhere
      in your code (a subscriber callback, a separate thread with proper
      locking, a state machine, etc.). The 20 Hz timer below detects the
      edge and acts on it.

    """

    def __init__(self):
        super().__init__("trajectory_action_client")

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
        self._flag = False
        self._prev_flag = False
        self._target_joint_positions = np.zeros(7)

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        self.gripper_action_client = ActionClient(
            self, ParallelGripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

        self._goal_handle = None
        self._goal_active = False

        self.create_timer(0.05, self._check_flag)

    def set_flag(self, value: bool, target_joint_positions: np.ndarray):
        self._flag = value
        self._target_joint_positions = target_joint_positions

    def _check_flag(self):
        if self._flag and not self._prev_flag:
            self._send_goal()
        elif not self._flag and self._prev_flag:
            self._cancel_goal()
        self._prev_flag = self._flag

    def _build_trajectory_goal(self) -> FollowJointTrajectory.Goal:

        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self._joint_names[:-1]

        point = JointTrajectoryPoint()
        point.positions = self._target_joint_positions[:-1]
        point.time_from_start = Duration(sec=5, nanosec=0)

        traj.points = [point]
        goal_msg.trajectory = traj
        return goal_msg

    def _send_goal(self):
        if self._goal_active:
            self.get_logger().warn("Goal already active, ignoring new send request")
            return

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available")
            return

        goal_msg = self._build_trajectory_goal()
        self.get_logger().info("Flag True -> sending trajectory goal")
        self._goal_active = True

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            self._goal_active = False
            self._goal_handle = None
            return

        self.get_logger().info("Goal accepted")
        self._goal_handle = goal_handle

        if not self._flag:
            self._cancel_goal()

        result_future = goal_handle.get_result_future()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        pass

    def _result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Trajectory finished: status={status}, error_code={result.error_code}")
        self._goal_active = False
        self._goal_handle = None

    def _cancel_goal(self):
        if self._goal_handle is None:
            self.get_logger().info("Flag False but no active goal to cancel")
            return

        self.get_logger().info("Flag False -> cancelling trajectory mid-motion")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Cancel accepted by action server")
        else:
            self.get_logger().warn("Cancel rejected by action server")


def main():
    rclpy.init()
    node = GatedTrajectoryClient()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
