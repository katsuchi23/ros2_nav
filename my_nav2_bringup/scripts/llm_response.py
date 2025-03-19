#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LLMResponder(Node):
    def __init__(self):
        super().__init__('llm_responder')
        # Subscribe to the gemini_camera topic
        self.camera_sub = self.create_subscription(String, 'gemini_camera', self.camera_callback, 10)
        # Publisher to send LLM responses on the llm_response topic
        self.llm_pub = self.create_publisher(String, 'llm_response', 10)

    def camera_callback(self, msg):
        self.get_logger().info(f"Received gemini_camera message: {msg.data}")
        # Simulate an LLM processing the image (here we simply output a dummy response)
        response = String()
        response.data = "Photo is received. LLM processed the image."
        self.llm_pub.publish(response)
        self.get_logger().info("LLM response published.")

def main(args=None):
    rclpy.init(args=args)
    node = LLMResponder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
