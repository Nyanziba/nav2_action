import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from voicevox_ros2_interface.msg import Speaker

class Nav2GoToPoseActionClient(Node):
    def __init__(self):
        super().__init__("nav2_call_client")
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.voice_count=0
        # voicevox_server
        self.voicevox_publisher = self.create_publisher(Speaker,'/voicevox_ros2/speaker',2)
        #
        # nav2_navtopose_action_server
        self.nav_to_pose_client = ActionClient(self,  NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Initialized action client 'nav2_call_client'")
        
    def go_to_pose(self, pose: PoseStamped, behavior_tree: str = '') -> bool:
        """Send a `NavigateToPose` action request."""
        
        self.get_logger().info("Waiting for 'NavigateToPose' action server...")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info(
            f'Navigating to goal: x={pose.pose.position.x}, y={pose.pose.position.y}...'
        )
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error(
                f'Goal to x={pose.pose.position.x}, y={pose.pose.position.y} was rejected!'
            )
            return False

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            voicevox_msg = Speaker()
            voicevox_msg.text = "navigation succeeded"
            voicevox_msg.id = 3
            self.voice_count = 0
            self.voicevox_publisher.publish(voicevox_msg)
        else:
            self.get_logger().info('Navigation failed!')

        return True

    def _feedback_callback(self, feedback_msg):
        nav_time = feedback_msg.feedback.navigation_time.sec
        
        if nav_time %  3 == 0:
            print(self.voice_count)
            if self.voice_count < nav_time:
                print(self.voice_count)
                self.voice_count += 3

                voicevox_msg = Speaker()
                voicevox_msg.text = "進行中"
                voicevox_msg.id = 3
                self.voicevox_publisher.publish(voicevox_msg)
        
 

def main(args=None):
    rclpy.init(args=args)
    action_client_node = Nav2GoToPoseActionClient()

    # ?????PoseStamped??????????
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = action_client_node.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = -1.0
    goal_pose.pose.orientation.w = 1.0

    # ??????????????????
    action_client_node.go_to_pose(goal_pose)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
