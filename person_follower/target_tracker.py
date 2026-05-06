import math
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import CompressedImage, LaserScan, Image
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler


@dataclass
class AlphaBeta:
    alpha: float = 0.25
    beta: float = 0.25**2 / 2
    x: float = None
    v: float = 0.0
    t: float = None

    def reset(self):
        self.x, self.v, self.t = None, 0.0, None

    def update(self, z: float, t_now: float = None) -> float:
        if t_now is None:
            t_now = time.time()

        if self.x is None or abs(z - self.x) > 2.0:
            self.x, self.v, self.t = z, 0.0, t_now
            return self.x

        dt = max(t_now - self.t, 1e-3)
        x_pred = self.x + self.v * dt
        v_pred = self.v
        r = z - x_pred

        self.x = x_pred + self.alpha * r
        self.v = v_pred + self.beta * r / dt
        self.t = t_now
        return self.x


class PositionFilter:
    def __init__(self, window_size=5, min_confidence=3):
        self.x_buffer = deque(maxlen=window_size)
        self.y_buffer = deque(maxlen=window_size)
        self.heading_buffer = deque(maxlen=window_size)
        self.distance_buffer = deque(maxlen=window_size)
        self.window_size = window_size
        self.min_confidence = min_confidence
        self.valid_measurements = 0
        self.last_valid_x = None
        self.last_valid_y = None
        self.last_valid_heading = None
        self.last_valid_distance = None

    def update(self, x, y, heading, distance):
        if not (math.isnan(x) or math.isnan(y) or math.isnan(heading) or math.isnan(distance)):
            self.x_buffer.append(x)
            self.y_buffer.append(y)
            self.heading_buffer.append(heading)
            self.distance_buffer.append(distance)
            self.valid_measurements += 1
            self.last_valid_x = x
            self.last_valid_y = y
            self.last_valid_heading = heading
            self.last_valid_distance = distance

        if len(self.x_buffer) >= self.window_size:
            self.valid_measurements = self.window_size

    def get_filtered_position(self):
        if self.valid_measurements >= self.min_confidence:
            x = float(np.median(self.x_buffer))
            y = float(np.median(self.y_buffer))
            heading = float(np.median(self.heading_buffer))
            distance = float(np.median(self.distance_buffer))
            return x, y, heading, distance, True

        if self.last_valid_x is not None:
            return (
                self.last_valid_x,
                self.last_valid_y,
                self.last_valid_heading,
                self.last_valid_distance,
                False,
            )

        return float('nan'), float('nan'), float('nan'), float('nan'), False

    def reset(self):
        self.x_buffer.clear()
        self.y_buffer.clear()
        self.heading_buffer.clear()
        self.distance_buffer.clear()
        self.valid_measurements = 0
        self.last_valid_x = None
        self.last_valid_y = None
        self.last_valid_heading = None
        self.last_valid_distance = None


class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')

        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('fallback_distance', 2.0)

        model_path = str(self.get_parameter('model_path').value)
        device = str(self.get_parameter('device').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.fallback_distance = float(self.get_parameter('fallback_distance').value)

        self.detector = YOLO(model_path, verbose=False).to(device)
        self.person_class_idx = 0

        self.lidar_data = None
        self.position_filter = PositionFilter(window_size=7, min_confidence=3)
        self.distance_filter = AlphaBeta(alpha=0.25)

        self.target_msg = Bool()
        self.target_msg.data = False

        self.fx = 196.32
        self.fy = 196.32
        self.cx = 127.18
        self.cy = 126.63

        self.create_subscription(
            CompressedImage,
            '/oakd/rgb/preview/image_raw/compressed',
            self.image_callback,
            10
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.person_loc_pub = self.create_publisher(
            PoseStamped,
            '/see/person_location',
            10
        )
        self.target_status_pub = self.create_publisher(
            Bool,
            '/see/target_in_frame',
            10
        )
        self.raw_image_vis_pub = self.create_publisher(
            Image,
            '/see/person_detector/image_raw',
            10
        )
        self.compressed_image_vis_pub = self.create_publisher(
            CompressedImage,
            '/see/person_detector/image_raw/compressed',
            10
        )
        self.distance_publisher = self.create_publisher(
            Float32,
            '/see/target_distance',
            10
        )
        self.heading_publisher = self.create_publisher(
            Float32,
            '/see/target_heading',
            10
        )

        self.get_logger().info('target_tracker started')

    def lidar_callback(self, msg: LaserScan):
        self.lidar_data = msg

    def compute_heading(self, center):
        x_norm = (center[0] - self.cx) / self.fx
        theta_cam = -math.atan(x_norm) 

        dx, dy = 0.0635, 0.0381
        x_c, y_c = math.cos(theta_cam), math.sin(theta_cam)
        theta_l = math.atan2(y_c - dy, x_c - dx)
        return theta_l

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        results = self.detector(frame, verbose=False)[0]

        best = self.select_target(results)

        if best is not None and self.lidar_data is not None:
            x1, y1, x2, y2, conf = best
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            heading = self.compute_heading((center_x, center_y))
            distance = self.get_lidar_distance(heading)

            if not math.isnan(distance):
                pos_x = distance * math.cos(heading)
                pos_y = distance * math.sin(heading)
            else:
                pos_x = self.fallback_distance * math.cos(heading)
                pos_y = self.fallback_distance * math.sin(heading)

            self.position_filter.update(pos_x, pos_y, heading, distance)
            filtered_x, filtered_y, filtered_heading, filtered_distance, has_confidence = (
                self.position_filter.get_filtered_position()
            )

            self.draw_target(
                frame,
                x1, y1, x2, y2,
                center_x, center_y,
                conf,
                filtered_x, filtered_y,
                filtered_heading,
                filtered_distance,
                has_confidence
            )

            ps_local = PoseStamped()
            ps_local.header.stamp = Time().to_msg()
            ps_local.header.frame_id = 'base_link'
            ps_local.pose.position.x = filtered_x
            ps_local.pose.position.y = filtered_y
            ps_local.pose.position.z = 0.0

            qx, qy, qz, qw = quaternion_from_euler(0, 0, filtered_heading)
            ps_local.pose.orientation.x = qx
            ps_local.pose.orientation.y = qy
            ps_local.pose.orientation.z = qz
            ps_local.pose.orientation.w = qw

            try:
                ps_global = self.tf_buffer.transform(
                    ps_local,
                    'odom',
                    timeout=Duration(seconds=0.2)
                )
            except TransformException:
                ps_global = ps_local

            self.person_loc_pub.publish(ps_global)
            self.target_msg.data = True

            if not math.isnan(filtered_distance):
                self.distance_publisher.publish(Float32(data=float(filtered_distance)))
            self.heading_publisher.publish(Float32(data=float(filtered_heading)))

        else:
            self.target_msg.data = False
            self.position_filter.reset()

        self.target_status_pub.publish(self.target_msg)

        raw_out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        compressed_out_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.raw_image_vis_pub.publish(raw_out_msg)
        self.compressed_image_vis_pub.publish(compressed_out_msg)

    def select_target(self, results):
        if results.boxes is None or len(results.boxes) == 0:
            return None

        boxes = results.boxes.xyxy.cpu().numpy()
        classes = results.boxes.cls.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()

        best = None
        max_area = 0.0

        for box, cls, score in zip(boxes, classes, scores):
            if int(cls) != self.person_class_idx:
                continue
            if float(score) < self.confidence_threshold:
                continue

            x1, y1, x2, y2 = box
            area = max(0.0, (x2 - x1) * (y2 - y1))

            if area > max_area:
                max_area = area
                best = (x1, y1, x2, y2, float(score))

        return best

    def draw_target(
        self,
        frame,
        x1, y1, x2, y2,
        center_x, center_y,
        conf,
        filtered_x, filtered_y,
        filtered_heading,
        filtered_distance,
        has_confidence
    ):
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

        text_x = x2 + 10
        distance_text = (
            'Distance: unknown'
            if math.isnan(filtered_distance)
            else f'Distance: {filtered_distance:.2f}m'
        )
        bearing_text = f'Bearing: {math.degrees(filtered_heading):.1f} deg'
        position_text = f'Position: ({filtered_x:.2f}, {filtered_y:.2f})'
        conf_text = f'YOLO: {conf:.2f}'
        quality_text = f'Filter: {"High" if has_confidence else "Low"}'

        cv2.putText(frame, 'PERSON', (text_x, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, conf_text, (text_x, y1 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, distance_text, (text_x, y1 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, bearing_text, (text_x, y1 + 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, position_text, (text_x, y1 + 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, quality_text, (text_x, y1 + 170), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def get_lidar_distance(self, angle):
        try:
            idx = int(math.degrees(angle) * 2 + 180)

            if self.lidar_data is None or idx < 0 or idx >= len(self.lidar_data.ranges):
                return float('nan')

            start_idx = max(0, idx - 20)
            end_idx = min(len(self.lidar_data.ranges), idx + 20)
            rngs = self.lidar_data.ranges[start_idx:end_idx]

            valid_ranges = [r for r in rngs if not math.isinf(r) and not math.isnan(r)]
            if not valid_ranges:
                return float('nan')

            min_distance = min(valid_ranges)
            distance_smoothed = self.distance_filter.update(min_distance + 0.1)
            return distance_smoothed

        except Exception:
            return float('nan')


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


if __name__ == '__main__':
    main()