import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_unstamped')
        self.declare_parameter('command_timeout', 0.75)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.20)
        self.declare_parameter('publish_rate', 20.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.last_cmd_time = None
        self.motion_allowed = False
        self.last_status_text = 'waiting for safety command'

        self.latest_cmd = Twist()
        self.latest_cmd.linear.x = 0.0
        self.latest_cmd.linear.y = 0.0
        self.latest_cmd.linear.z = 0.0
        self.latest_cmd.angular.x = 0.0
        self.latest_cmd.angular.y = 0.0
        self.latest_cmd.angular.z = 0.0

        self.create_subscription(
            TwistStamped,
            self.input_topic,
            self.cmd_cb,
            10
        )
        self.create_subscription(
            Bool,
            '/safety/motion_allowed',
            self.motion_allowed_cb,
            10
        )
        self.create_subscription(
            String,
            '/safety/status_text',
            self.status_text_cb,
            10
        )

        self.base_cmd_pub = self.create_publisher(
            Twist,
            self.output_topic,
            10
        )

        self.create_timer(1.0 / self.publish_rate, self.publish_to_base)

        self.get_logger().info(f'person_follower started: {self.input_topic} -> {self.output_topic}')

    def now_sec(self):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        return float(sec) + float(nsec) * 1e-9

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def cmd_cb(self, msg: TwistStamped):
        self.last_cmd_time = self.now_sec()

        self.latest_cmd.linear.x = self.clamp(
            msg.twist.linear.x,
            -self.max_linear_speed,
            self.max_linear_speed
        )
        self.latest_cmd.linear.y = 0.0
        self.latest_cmd.linear.z = 0.0

        self.latest_cmd.angular.x = 0.0
        self.latest_cmd.angular.y = 0.0
        self.latest_cmd.angular.z = self.clamp(
            msg.twist.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed
        )

    def motion_allowed_cb(self, msg: Bool):
        self.motion_allowed = msg.data

    def status_text_cb(self, msg: String):
        self.last_status_text = msg.data

    def command_is_fresh(self):
        if self.last_cmd_time is None:
            return False
        return (self.now_sec() - self.last_cmd_time) <= self.command_timeout

    def stop_robot(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.linear.y = 0.0
        stop.linear.z = 0.0
        stop.angular.x = 0.0
        stop.angular.y = 0.0
        stop.angular.z = 0.0
        self.base_cmd_pub.publish(stop)

    def publish_to_base(self):
        if not self.motion_allowed:
            self.stop_robot()
            return

        if not self.command_is_fresh():
            self.stop_robot()
            return

        self.base_cmd_pub.publish(self.latest_cmd)

    def destroy_node(self):
        try:
            self.stop_robot()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()