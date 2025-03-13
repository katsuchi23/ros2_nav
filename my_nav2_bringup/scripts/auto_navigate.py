#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Define a list of predefined waypoints
        self.waypoints = self.create_waypoints()
        self.current_index = 0
        self.send_next_goal()

    def create_waypoints(self):
        # Create a list of PoseStamped messages with predefined positions and orientations
        waypoints = []
        # Define your waypoints here as tuples:
        # (x, y, z, qx, qy, qz, qw)
        points = [
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),       # Waypoint 1
            (1.0, 0.0, 0.0, 0.0, 0.0, 0.707, 0.707),     # Waypoint 2
            (2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)          # Waypoint 3
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
            # Optionally, handle rejection (e.g., retry or log)
            return
        self.get_logger().info("Goal accepted, awaiting result.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if hasattr(result, 'error_code') and result.error_code != 0:
            self.get_logger().error(
                f"Navigation failed for waypoint {self.current_index + 1} with error code: {result.error_code}"
            )
            # Optionally, implement failure handling here (e.g., retry)
        else:
            self.get_logger().info(
                f"Navigation succeeded for waypoint {self.current_index + 1}."
            )
        self.current_index += 1
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
