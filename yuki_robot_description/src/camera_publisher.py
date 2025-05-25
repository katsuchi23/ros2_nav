import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info_corrected', 10)
        self.subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        msg.d = [0.1, -0.05, 0.001, 0.0015, -0.0012]
        msg.distortion_model = 'plumb_bob'
        msg.k = [554.254691191187, 0.0, 320.5,
                 0.0, 554.254691191187, 240.5,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [554.254691191187, 0.0, 320.5, 0.0,
                 0.0, 554.254691191187, 240.5, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.height = 480
        msg.width = 640
        msg.binning_x = 0
        msg.binning_y = 0
        msg.roi.x_offset = 0
        msg.roi.y_offset = 0
        msg.roi.height = 0
        msg.roi.width = 0
        msg.roi.do_rectify = True
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()
    rclpy.spin(camera_info_publisher)
    camera_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
