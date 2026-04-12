import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
import numpy as np


class RangeBearingEstimator(Node):
    def __init__(self):
        super().__init__('range_bearing_estimator')

        self.declare_parameter('alpha', 0.3)
        self.declare_parameter('depth_patch_size', 5)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 5.0)

        # Camera intrinsics
        self.fx = None
        self.cx = None

        # Filter state
        self.filtered_range   = None
        self.filtered_bearing = None

        # Latest data
        self.latest_depth  = None
        self.latest_target = None

        # Subscribers
        self.create_subscription(
            CameraInfo, '/oakd/rgb/preview/camera_info', self.camera_info_cb, 10)
        self.create_subscription(
            Image, '/oakd/rgb/preview/depth', self.depth_cb, 10)
        self.create_subscription(
            Detection2D, '/tracked_target', self.target_cb, 10)

        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, '/range_bearing', 10)

        self.create_timer(0.1, self.estimate)

        self.get_logger().info('RangeBearingEstimator started')

    def camera_info_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.get_logger().info(
                f'Camera intrinsics — fx={self.fx:.1f}, cx={self.cx:.1f}')

    def depth_cb(self, msg: Image):
        self.latest_depth = msg

    def target_cb(self, msg: Detection2D):
        self.latest_target = msg

    def get_depth_at(self, depth_msg, px, py):
        patch = self.get_parameter('depth_patch_size').value
        half  = patch // 2

        # Handle both float32 (metres) and uint16 (millimetres)
        if depth_msg.encoding == '32FC1':
            dtype = np.float32
            scale = 1.0
        else:
            dtype = np.uint16
            scale = 0.001

        arr = np.frombuffer(depth_msg.data, dtype=dtype).reshape(
            depth_msg.height, depth_msg.width)

        y1 = max(0, py - half)
        y2 = min(depth_msg.height, py + half + 1)
        x1 = max(0, px - half)
        x2 = min(depth_msg.width,  px + half + 1)

        patch_data = arr[y1:y2, x1:x2].flatten().astype(np.float32) * scale

        min_d = self.get_parameter('min_depth').value
        max_d = self.get_parameter('max_depth').value
        valid = patch_data[(patch_data > min_d) & (patch_data < max_d)]

        if len(valid) == 0:
            return None

        return float(np.median(valid))

    def estimate(self):
        if self.fx is None:
            self.get_logger().debug('Waiting for camera info...')
            return
        if self.latest_depth is None:
            self.get_logger().debug('Waiting for depth image...')
            return
        if self.latest_target is None:
            self.get_logger().debug('Waiting for tracked target...')
            return

        alpha = self.get_parameter('alpha').value

        px = int(self.latest_target.bbox.center.position.x)
        py = int(self.latest_target.bbox.center.position.y)

        range_m = self.get_depth_at(self.latest_depth, px, py)
        if range_m is None:
            self.get_logger().warn('No valid depth at target location')
            return

        # Bearing: positive = person is to the right
        bearing_rad = (px - self.cx) / self.fx

        # Low-pass filter
        if self.filtered_range is None:
            self.filtered_range   = range_m
            self.filtered_bearing = bearing_rad
        else:
            self.filtered_range   = (alpha * range_m
                                     + (1 - alpha) * self.filtered_range)
            self.filtered_bearing = (alpha * bearing_rad
                                     + (1 - alpha) * self.filtered_bearing)

        msg = Float32MultiArray()
        msg.data = [float(self.filtered_range), float(self.filtered_bearing)]
        self.pub.publish(msg)

        self.get_logger().info(
            f'range={self.filtered_range:.2f}m  bearing={self.filtered_bearing:.3f}rad')


def main(args=None):
    rclpy.init(args=args)
    node = RangeBearingEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()