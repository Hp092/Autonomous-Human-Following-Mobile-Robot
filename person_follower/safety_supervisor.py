import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import TwistStamped


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')

        self.declare_parameter('min_obstacle_distance', 0.50)
        self.declare_parameter('target_loss_timeout', 2.0)
        self.declare_parameter('camera_timeout', 1.0)
        self.declare_parameter('lidar_timeout', 1.0)
        self.declare_parameter('controller_timeout', 1.0)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.20)
        self.declare_parameter('scan_angle_min_deg', -25.0)
        self.declare_parameter('scan_angle_max_deg', 25.0)
        self.declare_parameter('publish_rate', 20.0)

        self.min_obstacle_distance = float(self.get_parameter('min_obstacle_distance').value)
        self.target_loss_timeout = float(self.get_parameter('target_loss_timeout').value)
        self.camera_timeout = float(self.get_parameter('camera_timeout').value)
        self.lidar_timeout = float(self.get_parameter('lidar_timeout').value)
        self.controller_timeout = float(self.get_parameter('controller_timeout').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.scan_angle_min_deg = float(self.get_parameter('scan_angle_min_deg').value)
        self.scan_angle_max_deg = float(self.get_parameter('scan_angle_max_deg').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.last_camera_time = None
        self.last_lidar_time = None
        self.last_controller_time = None
        self.last_target_seen_time = None

        self.target_detected = False
        self.obstacle_too_close = False
        self.safety_hold = False
        self.last_fault_reason = 'system startup'

        self.requested_cmd = TwistStamped()

        self.create_subscription(
            TwistStamped,
            '/think/follow_cmd_vel',
            self.follow_cmd_cb,
            10
        )
        self.create_subscription(
            Bool,
            '/see/target_in_frame',
            self.target_flag_cb,
            10
        )
        self.create_subscription(
            Float32,
            '/see/target_distance',
            self.target_distance_cb,
            10
        )
        self.create_subscription(
            Float32,
            '/see/target_heading',
            self.target_heading_cb,
            10
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_cb,
            10
        )
        self.create_subscription(
            CompressedImage,
            '/oakd/rgb/preview/image_raw/compressed',
            self.camera_cb,
            10
        )

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/safety/motion_allowed', 10)
        self.reason_pub = self.create_publisher(String, '/safety/status_text', 10)

        self.create_timer(1.0 / self.publish_rate, self.safety_loop)

        self.get_logger().info('safety_supervisor started')

    def now_sec(self):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        return float(sec) + float(nsec) * 1e-9

    def follow_cmd_cb(self, msg: TwistStamped):
        self.last_controller_time = self.now_sec()
        self.requested_cmd = msg

    def target_flag_cb(self, msg: Bool):
        self.target_detected = msg.data
        if msg.data:
            self.last_target_seen_time = self.now_sec()

    def target_distance_cb(self, msg: Float32):
        return

    def target_heading_cb(self, msg: Float32):
        return

    def camera_cb(self, msg: CompressedImage):
        self.last_camera_time = self.now_sec()

    def lidar_cb(self, msg: LaserScan):
        self.last_lidar_time = self.now_sec()
        self.obstacle_too_close = self.check_forward_obstacle(msg)

    def check_forward_obstacle(self, scan: LaserScan):
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges

        front_min = math.radians(self.scan_angle_min_deg)
        front_max = math.radians(self.scan_angle_max_deg)

        nearest = float('inf')

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if angle < front_min or angle > front_max:
                continue
            if math.isinf(r) or math.isnan(r):
                continue
            if r < nearest:
                nearest = r

        if nearest < self.min_obstacle_distance:
            return True
        return False

    def stale(self, last_time, timeout):
        if last_time is None:
            return True
        return (self.now_sec() - last_time) > timeout

    def controller_command_is_safe(self):
        vx = self.requested_cmd.twist.linear.x
        wz = self.requested_cmd.twist.angular.z

        if abs(vx) > self.max_linear_speed:
            self.last_fault_reason = 'linear velocity limit exceeded'
            return False

        if abs(wz) > self.max_angular_speed:
            self.last_fault_reason = 'angular velocity limit exceeded'
            return False

        return True

    def safety_conditions_ok(self):
        if self.stale(self.last_camera_time, self.camera_timeout):
            self.last_fault_reason = 'camera data timeout'
            return False

        if self.stale(self.last_lidar_time, self.lidar_timeout):
            self.last_fault_reason = 'lidar data timeout'
            return False

        if self.stale(self.last_controller_time, self.controller_timeout):
            self.last_fault_reason = 'follow controller timeout'
            return False

        if self.obstacle_too_close:
            self.last_fault_reason = 'obstacle inside safety distance'
            return False

        if self.last_target_seen_time is None:
            self.last_fault_reason = 'target not acquired yet'
            return False

        if (self.now_sec() - self.last_target_seen_time) > self.target_loss_timeout:
            self.last_fault_reason = 'target lost for too long'
            return False

        if not self.controller_command_is_safe():
            return False

        return True

    def publish_stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def publish_requested_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.requested_cmd.twist.linear.x
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.requested_cmd.twist.angular.z
        self.cmd_pub.publish(msg)

    def safety_loop(self):
        safe_now = self.safety_conditions_ok()

        if not safe_now:
            self.safety_hold = True
            self.publish_stop()
            self.status_pub.publish(Bool(data=False))
            self.reason_pub.publish(String(data=f'STOP: {self.last_fault_reason}'))
            return

        if self.safety_hold:
            recovered = (
                not self.stale(self.last_camera_time, self.camera_timeout)
                and not self.stale(self.last_lidar_time, self.lidar_timeout)
                and not self.stale(self.last_controller_time, self.controller_timeout)
                and not self.obstacle_too_close
                and self.target_detected
                and self.last_target_seen_time is not None
                and (self.now_sec() - self.last_target_seen_time) <= self.target_loss_timeout
                and self.controller_command_is_safe()
            )

            if not recovered:
                self.publish_stop()
                self.status_pub.publish(Bool(data=False))
                self.reason_pub.publish(String(data='STOP: waiting for safe recovery'))
                return

            self.safety_hold = False

        self.publish_requested_cmd()
        self.status_pub.publish(Bool(data=True))
        self.reason_pub.publish(String(data='OK: motion allowed'))


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()