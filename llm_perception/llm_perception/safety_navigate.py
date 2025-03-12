import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from .model import Model
import sys

class BehaviorTreeLogListener(Node):
    def __init__(self):
        super().__init__('behavior_tree_log_listener')
        self.get_logger().info("Initializing Model")
        print(sys.executable)
        self.model = Model()

        # Use a custom QoS with increased depth to buffer more messages
        qos_profile = QoSProfile(depth=100)
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info("Subscribed to /behavior_tree_log with QoS depth 100.")

        # Subscriber for current pose from AMCL
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos_profile
        )
        self.get_logger().info("Subscribed to /amcl_pose.")

        # Publisher for goal_pose messages
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_logger().info("Goal publisher created on topic 'goal_pose'.")

        # Define the hard-coded safety point (fallback)
        self.safety_point = PoseStamped()
        self.safety_point.header.frame_id = 'map'
        self.safety_point.pose.position.x = -1.95 # defaut value
        self.safety_point.pose.position.y = -0.05
        self.safety_point.pose.position.z = 0.0
        self.safety_point.pose.orientation.x = 0.0
        self.safety_point.pose.orientation.y = 0.0
        self.safety_point.pose.orientation.z = 0.0
        self.safety_point.pose.orientation.w = 1.0

        # Variable to hold the latest pose from /amcl_pose
        self.current_pose = None

    def pose_callback(self, msg):
        # Store the latest pose from AMCL
        self.current_pose = msg

    def listener_callback(self, msg):
        # Loop over all events in the BehaviorTreeLog message
        for event in msg.event_log:
            if event.node_name == "NavigateWithReplanning" and event.current_status == "FAILURE":
                self.get_logger().info("Goal Failed")
                # Log the current robot position from /amcl_pose if available
                if self.current_pose is not None:
                    self.get_logger().info("Navigating to nearest safety point")
                    self.get_logger().info("Thinking...")
                    x = self.current_pose.pose.pose.position.x
                    y = self.current_pose.pose.pose.position.y
                    self.get_logger().info(f"Current robot position: x={x}, y={y}")
                    ans = self.model.response((x, y))
                    self.safety_point.pose.position.x = ans[0]
                    self.safety_point.pose.position.y = ans[1]
                    self.get_logger().info(f"Finished thinking, navigating to safety point: x={ans[0]}, y={ans[1]}")
                else:
                    self.get_logger().warn("Current pose not available from /amcl_pose.")

                # Publish the hard-coded safety point regardless
                self.safety_point.header.stamp = self.get_clock().now().to_msg()
                
                self.goal_pub.publish(self.safety_point)
                self.get_logger().info(
                        f"Published current pose as safety point: x={self.safety_point.pose.position.x}, y={self.safety_point.pose.position.y}"
                    )
            elif event.node_name == "NavigateWithReplanning" and event.current_status == "SUCCESS":
                self.get_logger().info("Goal Success.")
            else:
                self.get_logger().debug(
                    f"Node: {event.node_name}, Previous: {event.previous_status}, Current: {event.current_status}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeLogListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
