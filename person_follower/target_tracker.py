import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Bool


class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')

        self.declare_parameter('iou_threshold', 0.3)
        self.declare_parameter('lost_timeout', 1.0)

        # State
        self.tracked_box = None
        self.last_seen_time = None

        # Subscribers
        self.create_subscription(Detection2D, '/detections', self.detection_cb, 10)

        # Publishers
        self.tracked_pub = self.create_publisher(Detection2D, '/tracked_target', 10)
        self.lost_pub    = self.create_publisher(Bool, '/target_lost', 10)

        # Timeout checker
        self.create_timer(0.1, self.check_timeout)

        self.get_logger().info('TargetTracker started')

    def compute_iou(self, box_a, box_b):
        def corners(box):
            cx = box.bbox.center.position.x
            cy = box.bbox.center.position.y
            hw = box.bbox.size_x / 2.0
            hh = box.bbox.size_y / 2.0
            return cx - hw, cy - hh, cx + hw, cy + hh

        ax1, ay1, ax2, ay2 = corners(box_a)
        bx1, by1, bx2, by2 = corners(box_b)

        ix1 = max(ax1, bx1)
        iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)

        if ix2 <= ix1 or iy2 <= iy1:
            return 0.0

        intersection = (ix2 - ix1) * (iy2 - iy1)
        area_a = (ax2 - ax1) * (ay2 - ay1)
        area_b = (bx2 - bx1) * (by2 - by1)
        union  = area_a + area_b - intersection

        return intersection / union if union > 0 else 0.0

    def detection_cb(self, msg: Detection2D):
        iou_threshold = self.get_parameter('iou_threshold').value

        # First ever detection — lock on
        if self.tracked_box is None:
            self.tracked_box = msg
            self.last_seen_time = self.get_clock().now()
            self.get_logger().info('Target acquired!')
            self.tracked_pub.publish(msg)
            return

        # Check if this detection matches tracked target
        iou = self.compute_iou(self.tracked_box, msg)

        if iou >= iou_threshold:
            # Same person — update
            self.tracked_box = msg
            self.last_seen_time = self.get_clock().now()
            self.tracked_pub.publish(msg)

            lost_msg = Bool()
            lost_msg.data = False
            self.lost_pub.publish(lost_msg)
        else:
            self.get_logger().debug(f'Ignored detection — IoU {iou:.2f} below threshold')

    def check_timeout(self):
        if self.last_seen_time is None:
            return

        timeout = self.get_parameter('lost_timeout').value
        elapsed = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9

        if elapsed > timeout:
            if self.tracked_box is not None:
                self.get_logger().warn(f'Target lost — no detection for {elapsed:.1f}s')
                self.tracked_box = None

            lost_msg = Bool()
            lost_msg.data = True
            self.lost_pub.publish(lost_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()