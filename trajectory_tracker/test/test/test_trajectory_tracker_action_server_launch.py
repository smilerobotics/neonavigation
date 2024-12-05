import unittest
from typing import Optional

import launch_testing.markers
import pytest
import rclpy
import rclpy.client
import rclpy.clock
import rclpy.node
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
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

    @classmethod
    def setUpClass(cls) -> None:
        print("TestActionServer.__init__")
        rclpy.init()

    def setUp(self) -> None:
        self._node = rclpy.node.Node("test_action_server_" + str(self._testMethodName))
        self._action_client = ActionClient(self._node, FollowPath, "follow_path")
        self._odom_pub = self._node.create_publisher(Odometry, "odom", 10)
        self._result: Optional[FollowPath.Result] = None
        self._odom_list: Optional[list[Pose]] = None

    def tearDown(self) -> None:
        self._odom_pub.destroy()
        self._action_client.destroy()
        self._node.destroy_node()

    def publish_current_odom(self) -> None:
        if self._odom_list:
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.header.stamp = self._node.get_clock().now().to_msg()
            # keep the last odom pose
            if len(self._odom_list) != 1:
                odom.pose.pose = self._odom_list.pop(0)
            else:
                odom.pose.pose = self._odom_list[0]

            self._odom_pub.publish(odom)

    def feedback_cb(self, feedback: FollowPath.Impl.FeedbackMessage) -> None:
        distance_to_goal = feedback.feedback.distance_to_goal
        self._node.get_logger().info(f"Remaining distance: {distance_to_goal}")

    def send_goal_and_wait_result(
        self, odom_list: list[Pose], path_poses: list[PoseStamped]
    ) -> int:
        self.assertTrue(self._action_client.wait_for_server(1.0))

        goal = FollowPath.Goal()
        goal.path = Path()
        goal.path.header.frame_id = "odom"
        goal.path.header.stamp = self._node.get_clock().now().to_msg()
        goal.path.poses = path_poses
        for pose in goal.path.poses:
            pose.header = goal.path.header

        self._odom_list = odom_list

        timer = self._node.create_timer(0.05, self.publish_current_odom)
        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        rclpy.spin_until_future_complete(self._node, send_goal_future, timeout_sec=1.0)
        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=3.0)
        self.assertTrue(result_future.done())
        timer.cancel()

        return result_future.result().status  # type: ignore[no-any-return]

    # __constants = {
    #     'STATUS_UNKNOWN': 0,
    #     'STATUS_ACCEPTED': 1,
    #     'STATUS_EXECUTING': 2,
    #     'STATUS_CANCELING': 3,
    #     'STATUS_SUCCEEDED': 4,
    #     'STATUS_CANCELED': 5,
    #     'STATUS_ABORTED': 6,
    # }

    def test_client_success(self) -> None:
        odom_list = []
        for i in range(11):
            odom_pose = Pose()
            odom_pose.position.x = i * 0.1 + 0.03
            if odom_pose.position.x > 1.0:
                odom_pose.position.x = 1.0
            odom_pose.orientation.w = 1.0
            odom_list.append(odom_pose)

        posees_list = []
        for i in range(11):
            path_pose = PoseStamped()
            path_pose.pose.position.x = i * 0.1
            path_pose.pose.orientation.w = 1.0
            posees_list.append(path_pose)

        status = self.send_goal_and_wait_result(odom_list, posees_list)
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)

    def test_client_unabled_to_follow_path(self) -> None:
        odom_list = []
        odom_pose = Pose()
        odom_pose.position.x = 0.1
        odom_pose.position.y = 0.5
        odom_list.append(odom_pose)

        posees_list = []
        for i in range(11):
            path_pose = PoseStamped()
            path_pose.pose.position.x = i * 0.1
            path_pose.pose.orientation.w = 1.0
            posees_list.append(path_pose)

        status = self.send_goal_and_wait_result(odom_list, posees_list)
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
