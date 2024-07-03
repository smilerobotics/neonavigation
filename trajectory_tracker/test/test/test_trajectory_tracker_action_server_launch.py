import time
import unittest
from typing import Optional

import launch_testing.markers
import pytest
import rclpy
import rclpy.client
import rclpy.clock
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient


@pytest.mark.launch_test
@launch_testing.markers.keep_alive  # type: ignore[misc]
def generate_test_description() -> LaunchDescription:
    node_log_level = [
        TextSubstitution(text="trajectory_tracker:="),
        LaunchConfiguration("node_log_level", default="info"),
    ]
    params_file = PathJoinSubstitution(
        [
            get_package_share_directory("trajectory_tracker"),  # type: ignore[no-untyped-call]
            "test",
            "configs",
            LaunchConfiguration("param_file", default="test_action_server.yaml"),
        ]
    )
    trajectory_tracker_cmd = Node(
        package="trajectory_tracker",
        executable="trajectory_tracker",
        name="trajectory_tracker",
        output="screen",
        arguments=["--ros-args", "--log-level", node_log_level],
        parameters=[params_file],
    )
    return LaunchDescription(
        [
            trajectory_tracker_cmd,
            ReadyToTest(),  # type: ignore[no-untyped-call]
        ]
    )


class TestActionServer(unittest.TestCase):

    def __init__(self, methodName: str = "runTest") -> None:
        super().__init__(methodName)
        rclpy.init()
        self._node = rclpy.node.Node("test_trajectory_tracker_action_server")
        self._action_client = ActionClient(self._node, FollowPath, "follow_path")
        self._odom_pub = self._node.create_publisher(Odometry, "odom", 10)
        self._x_pos = 0.03
        self._result: Optional[FollowPath.Result] = None

    def publish_current_odom(self) -> None:
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = self._node.get_clock().now().to_msg()
        odom.pose.pose.position.x = self._x_pos
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)
        self._x_pos += 0.1
        if self._x_pos > 1.0:
            self._x_pos = 1.0

    def feedback_cb(self, feedback: FollowPath.Impl.FeedbackMessage) -> None:
        distance_to_goal = feedback.feedback.distance_to_goal
        self._node.get_logger().info(f"Remaining distance: {distance_to_goal}")

    def goal_response_callback(self, future: rclpy.client.Future) -> None:
        self._result = future.result()
        self._node.get_logger().info("Goal received")

    def test_client_connection(self) -> None:
        self._node.get_logger().info("Waiting for action server")
        self.assertTrue(self._action_client.wait_for_server(1.0))

        goal = FollowPath.Goal()
        goal.path = Path()
        goal.path.header.frame_id = "odom"
        goal.path.header.stamp = self._node.get_clock().now().to_msg()
        goal.path.poses = []
        for i in range(11):
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = i * 0.1
            pose.pose.orientation.w = 1.0
            goal.path.poses.append(pose)

        self._node.create_timer(0.05, self.publish_current_odom)
        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        rclpy.spin_until_future_complete(self._node, send_goal_future)
        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted)

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self._node, result_future)
        self.assertTrue(result_future.done())
        for i in range(10):
            rclpy.spin_once(self._node)
            time.sleep(0.1)
            if self._result:
                break

        self.assertTrue(self._result is not None)
        if self._result:
            self.assertTrue(self._result.result is not None)
