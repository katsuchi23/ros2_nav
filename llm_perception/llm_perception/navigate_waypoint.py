import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from itertools import islice
from ament_index_python.packages import get_package_share_directory
import os

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Using NavigateToPose instead of NavigateThroughPoses
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.dir = get_package_share_directory('llm_perception')
        self.waypoints_file = os.path.join(self.dir, 'config', 'waypoints.txt')
        self.safety_point, self.waypoints = self.read_waypoints()
        self.current_index = 0
        self.send_next_goal()

    def read_waypoints(self):
        waypoints = []
        try:
            with open(self.waypoints_file, 'r') as file:
                for line in islice(file, 1, None):  # Skip the first line
                    if line.strip() and not line.startswith('#'):
                        x, y, z, qx, qy, qz, qw = map(float, line.split())
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
            # Use the first waypoint as the safety point and the remaining as navigation goals.
            safety_point = waypoints[0]
            goals = waypoints[1:]
        except FileNotFoundError:
            self.get_logger().error(f"Waypoints file '{self.waypoints_file}' not found.")
            safety_point, goals = None, []
        except ValueError as e:
            self.get_logger().error(f"Error parsing waypoints: {e}")
            safety_point, goals = None, []
        return safety_point, goals
    
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
            self.get_logger().error("Goal was rejected by the action server. Sending safety goal.")
            self.send_safety_goal()
            return
        self.get_logger().info("Goal accepted, awaiting result.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if hasattr(result, 'error_code') and result.error_code != 0:
            self.get_logger().error(
                f"Navigation failed for waypoint {self.current_index + 1} with error_code: {result.error_code}"
            )
            self.send_safety_goal()
            return
        else:
            self.get_logger().info(
                f"Navigation succeeded for waypoint {self.current_index + 1}."
            )
        self.current_index += 1
        self.send_next_goal()
    
    def send_safety_goal(self):
        # Send the safety goal defined as the first waypoint.
        self.get_logger().info("Sending safety point goal.")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.safety_point

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.safety_goal_response_callback)

    def safety_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Safety goal was rejected by the action server.")
            return
        self.get_logger().info("Safety goal accepted, awaiting result.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.safety_result_callback)

    def safety_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Safety navigation completed with result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
