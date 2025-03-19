#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.msg import BehaviorTreeLog
from rclpy.qos import QoSProfile
import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Action client for navigation goals
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher to publish a dummy camera message on goal failure/cancellation
        self.camera_pub = self.create_publisher(String, 'gemini_camera', 10)
        
        # Subscriber to get LLM responses and log them
        self.llm_sub = self.create_subscription(String, 'llm_response', self.llm_response_callback, 10)
        
        # Subscriber to monitor robot position from AMCL
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, 10)
        
        # Subscriber to BehaviorTreeLog (to detect goal success or failure)
        qos_profile = QoSProfile(depth=100)
        self.bt_log_sub = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info("Subscribed to /behavior_tree_log with QoS depth 100.")
        
        # Timer to periodically check if the robot is stuck (every 1 second)
        self.timer = self.create_timer(1.0, self.check_motion_callback)
        
        # Predefined waypoints
        self.waypoints = self.create_waypoints()
        self.current_index = 0
        
        # Variables for goal tracking and motion monitoring
        self.current_goal_handle = None
        self.goal_active = False
        self.goal_finished = False   # Set to True when the goal completes successfully
        self.goal_failed = False     # Set to True when the goal is marked as failed
        self.last_pose = None
        self.last_movement_time = None
        self.pose_threshold = 0.05   # Minimum distance (meters) to consider as movement
        self.goal_timeout = 10.0     # Time (seconds) without movement before marking goal as failed
        
        # Flag to stop navigation when a goal failure is detected
        self.stop_navigation = False
        
        self.send_next_goal()

    def create_waypoints(self):
        waypoints = []
        # Each tuple represents: (x, y, z, qx, qy, qz, qw)
        points = [
            (0.37900936935275825, -0.53, 0.0, 0.0, 0.0, 0.0, 1.0),
            (0.37900936935275825, -0.53, 0.0, 0.0, 0.0, 0.6030795513532161, 0.7976810482514948),
            (0.5295722339689427, 0.2063229200183943, 0.0, 0.0, 0.0, 0.6030795513532161, 0.7976810482514948),
            (-1.8886416154836014, 0.24392118516300115, 0.0, 0.0, 0.0, 0.9830641585295858, 0.1832617259945928),
        ]
        for x, y, z, qx, qy, qz, qw in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            waypoints.append(pose)
        return waypoints

    def send_next_goal(self):
        if self.stop_navigation:
            self.get_logger().info("Navigation has been stopped due to goal failure. Waiting for LLM response.")
            return

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints have been navigated.")
            return

        goal_pose = self.waypoints[self.current_index]
        self.get_logger().info(f"Sending waypoint {self.current_index + 1} of {len(self.waypoints)}")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the action server.")
            if not self.stop_navigation:
                self.current_index += 1
                self.send_next_goal()
            return

        self.get_logger().info("Goal accepted, waiting for BT log to determine outcome.")
        self.current_goal_handle = goal_handle
        self.goal_active = True
        self.goal_finished = False
        self.goal_failed = False
        
        # Reset the pose monitoring variables
        self.last_pose = None
        self.last_movement_time = self.get_clock().now()

    def listener_callback(self, msg):
        # Loop over all events in the BehaviorTreeLog message
        for event in msg.event_log:
            if event.node_name == "NavigateWithReplanning":
                if event.current_status == "FAILURE":
                    self.get_logger().info("BT Log indicates goal FAILURE.")
                    if self.goal_active and not self.goal_failed:
                        self.goal_failed = True
                        self.stop_navigation = True  # Stop any further navigation
                        self.cancel_current_goal()
                if event.current_status == "SUCCESS":
                    self.get_logger().info("BT Log indicates goal SUCCESS.")
                    if self.goal_active and not self.goal_finished and not self.stop_navigation:
                        self.goal_finished = True
                        self.goal_active = False
                        self.current_index += 1
                        self.send_next_goal()

    def amcl_pose_callback(self, msg):
        # Extract current robot position from AMCL message
        current_position = msg.pose.pose.position
        
        # If no previous pose is stored, initialize it
        if self.last_pose is None:
            self.last_pose = current_position
            self.last_movement_time = self.get_clock().now()
            return

        # Compute Euclidean distance between last known position and current position
        dx = current_position.x - self.last_pose.x
        dy = current_position.y - self.last_pose.y
        dz = current_position.z - self.last_pose.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        if distance > self.pose_threshold:
            # Robot moved – update the time and position
            self.last_movement_time = self.get_clock().now()
            self.last_pose = current_position

    def check_motion_callback(self):
        # If a goal is active, check if the robot appears stuck
        if self.goal_active and self.last_movement_time is not None:
            current_time = self.get_clock().now()
            elapsed = (current_time - self.last_movement_time).nanoseconds / 1e9
            if elapsed > self.goal_timeout:
                if self.goal_finished:
                    self.get_logger().info("Goal finished despite no recent movement.")
                else:
                    self.get_logger().warn("Robot appears to be stuck. Marking goal as failed.")
                    self.goal_failed = True
                    self.stop_navigation = True  # Stop further navigation
                    self.cancel_current_goal()

    def cancel_current_goal(self):
        if self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.goal_active = False

    def cancel_done_callback(self, future):
        self.get_logger().info("Goal cancelled due to failure/inactivity.")
        self.handle_failure_flow()

    def handle_failure_flow(self):
        # Publish a dummy message on the gemini_camera topic (failure flow)
        camera_msg = String()
        camera_msg.data = "Dummy camera message after goal failure/cancellation."
        self.get_logger().info("Sending dummy camera message after goal failure/cancellation.")
        self.camera_pub.publish(camera_msg)
        if self.stop_navigation:
            self.get_logger().info("Navigation halted due to goal failure. No further goals will be sent.")
            return
        self.current_index += 1
        self.send_next_goal()

    def llm_response_callback(self, msg):
        self.get_logger().info(f"LLM response received: {msg.data}")

    def destroy_node(self):
        self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
