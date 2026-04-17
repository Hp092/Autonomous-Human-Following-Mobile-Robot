import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped


class StationaryDemo(Node):
    def __init__(self):
        super().__init__('stationary_demo')

        # Send stop command to follow controller
        self.ctrl_pub = self.create_publisher(String, '/think/ctrl_cmd', 10)

        # Also publish zero velocity directly as backup
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Send stop every second
        self.create_timer(1.0, self.send_stop)

        self.get_logger().info('StationaryDemo started — robot locked in place')
        self.get_logger().info('YOLO detections visible on /see/person_detector/image_raw')

    def send_stop(self):
        # Tell follow controller to go idle
        stop_cmd = String()
        stop_cmd.data = 'stop'
        self.ctrl_pub.publish(stop_cmd)

        # Also publish zero velocity directly
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        self.get_logger().debug('Stop command sent')


def main(args=None):
    rclpy.init(args=args)
    node = StationaryDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()