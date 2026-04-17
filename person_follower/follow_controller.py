import math
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, UInt8, String, Float32
from geometry_msgs.msg import PoseStamped, TwistStamped


class State(Enum):
    IDLE = 0
    FOLLOWING = 1


class FollowController(Node):
    def __init__(self):
        super().__init__('follow_controller')

        self.declare_parameter('follow_distance', 1.0)
        self.declare_parameter('search_rot_speed', 0.4)
        self.declare_parameter('kp_linear', 0.45)
        self.declare_parameter('kp_angular', 0.45)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('deadband_angle_deg', 4.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('min_valid_distance', 0.2)
        self.declare_parameter('max_valid_distance', 5.0)
        self.declare_parameter('target_lost_timeout', 0.5)

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.search_rot_speed = float(self.get_parameter('search_rot_speed').value)
        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.deadband_angle = math.radians(float(self.get_parameter('deadband_angle_deg').value))
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.min_valid_distance = float(self.get_parameter('min_valid_distance').value)
        self.max_valid_distance = float(self.get_parameter('max_valid_distance').value)
        self.target_lost_timeout = float(self.get_parameter('target_lost_timeout').value)

        self.state = State.FOLLOWING

        self.target_detected = False
        self.target_pose = None
        self.target_distance = 0.0
        self.target_heading = 0.0
        self.last_seen_bearing = 0.0
        self.prev_angular_z = 0.0
        self.angular_smoothing = 0.2
        self.last_target_seen_time = None

        self.create_subscription(Bool, '/see/target_in_frame', self.target_flag_cb, 10)
        self.create_subscription(PoseStamped, '/see/person_location', self.target_pose_cb, 10)
        self.create_subscription(Float32, '/see/target_distance', self.target_distance_cb, 10)
        self.create_subscription(Float32, '/see/target_heading', self.target_heading_cb, 10)
        self.create_subscription(String, '/think/ctrl_cmd', self.ctrl_cmd_cb, 10)

        self.state_pub = self.create_publisher(UInt8, '/think/planner_state', 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/think/follow_cmd_vel', 10)

        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('follow_controller started')
        self.get_logger().info('Initial state set to FOLLOWING')

    def now_sec(self):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        return float(sec) + float(nsec) * 1e-9

    def target_flag_cb(self, msg: Bool):
        self.target_detected = msg.data
        if msg.data:
            self.last_target_seen_time = self.now_sec()

    def target_pose_cb(self, msg: PoseStamped):
        self.target_pose = msg

    def target_distance_cb(self, msg: Float32):
        self.target_distance = msg.data

    def target_heading_cb(self, msg: Float32):
        self.target_heading = msg.data
        self.last_seen_bearing = msg.data

    def ctrl_cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'follow':
            self.state = State.FOLLOWING
            self.get_logger().info('State changed to FOLLOWING')

        elif cmd == 'stop':
            self.state = State.IDLE
            self.publish_cmd_vel(0.0, 0.0)
            self.get_logger().info('State changed to IDLE')

    def target_recently_seen(self):
        if self.last_target_seen_time is None:
            return False
        return (self.now_sec() - self.last_target_seen_time) <= self.target_lost_timeout

    def distance_is_valid(self, distance: float) -> bool:
        if math.isnan(distance):
            return False
        if distance < self.min_valid_distance:
            return False
        if distance > self.max_valid_distance:
            return False
        return True

    def control_loop(self):
        if self.state == State.IDLE:
            self.publish_cmd_vel(0.0, 0.0)
            self.publish_state()
            return

        target_available = self.target_detected or self.target_recently_seen()

        if not target_available:
            spin_dir = 1.0 if self.last_seen_bearing >= 0.0 else -1.0
            self.publish_cmd_vel(0.0, spin_dir * self.search_rot_speed)
            self.publish_state()
            return

        if not self.distance_is_valid(self.target_distance):
            self.publish_cmd_vel(0.0, 0.0)
            self.publish_state()
            return

        range_error = self.target_distance - self.follow_distance
        angle_error = self.target_heading

        linear_x = self.kp_linear * range_error
        angular_z = self.kp_angular * angle_error

        if abs(angle_error) < self.deadband_angle:
            angular_z = 0.0

        linear_x *= max(0.2, math.cos(angle_error))
        linear_x = max(0.0, linear_x)

        angular_z = (
            self.angular_smoothing * angular_z
            + (1.0 - self.angular_smoothing) * self.prev_angular_z
        )

        linear_x = max(0.0, min(self.max_linear_speed, linear_x))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

        self.prev_angular_z = angular_z
        self.publish_cmd_vel(linear_x, angular_z)
        self.publish_state()

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_state(self):
        self.state_pub.publish(UInt8(data=self.state.value))


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()